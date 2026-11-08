/*
 * sdr-receiver-hpsdr2 - openHPSDR Ethernet Protocol 2 receiver for the
 * Red Pitaya / TRX-duo (Zynq 7010).
 *
 * Reuses the same FPGA DSP as the Protocol-1 receiver (8 DDCs, shared CIC rate).
 * This server speaks Protocol 2 (multi-port, one UDP stream per DDC).
 *
 * Verified against linhpsdr and Thetis source:
 *   - clients send the DDC frequency as a PHASE WORD computed for a 122.88 MHz
 *     DSP clock; the RP NCO runs at 125 MHz, so we rescale x122.88/125.
 *   - clients demux DDC data by UDP SOURCE port (1035+n), so each DDC streams
 *     from its own bound socket.
 *   - discovery reply: board type (Angelia=3) in byte 11, #DDCs in byte 20.
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

/* ---- configuration (extrawide: 4 DDC, 3-stage FIR chain, 1536 kHz max) ---- */
#define NUM_DDC            4       /* DDCs implemented in the FPGA           */
#define BOARD_TYPE         3       /* linhpsdr device enum: Angelia (2 ADCs) */
#define CODE_VERSION       1       /* firmware/code version (discovery [13]) */
#define PROTOCOL_VERSION   39      /* openHPSDR protocol version *10 ([12])  */
#define SAMPLES_PER_FRAME  238     /* 24-bit I/Q pairs per DDC packet        */
#define FIFO_WORD          32      /* bytes per instant in the DDR ring: 8 * 4B LE
                                      channels (DDC0_I, DDC0_Q, DDC1_I ... DDC3_Q),
                                      each a 26-bit signed value in a 32-bit LE word */
#define RATE_BASE          76800   /* rx_rate = RATE_BASE/rate_khz. = 125000 * the
                                      3-stage (24/25)(4/5)(4/5)=0.6144 FIR gain, so
                                      rate_khz=1536 -> R=50 (the CIC MINIMUM_RATE); the
                                      48/96/192/384/768/1536 family maps R=1600..50. */

#define ADC_CLOCK          125000000.0
#define HPSDR_DSP_CLOCK    122880000.0    /* clients hardcode this           */

/* ---- Protocol-2 UDP ports ---- */
#define PORT_COMMAND       1024    /* discovery/general/command in; reply out */
#define PORT_DDC_SPECIFIC  1025    /* DDC-specific in; high-priority status out */
#define PORT_MIC           1026
#define PORT_HIGH_PRIORITY 1027    /* high-priority in (run + phase words)     */
#define PORT_DDC_DATA0     1035    /* DDC n I/Q out from source port 1035+n    */

/* ---- Protocol-2 wideband (WB) display: raw ADC0 samples OUT ----
   The radio sends the full-rate ADC panorama to the host from UDP SOURCE port 1027
   (linhpsdr WIDE_BAND_TO_HOST_PORT); the host enables it via General-packet byte[23]&1.
   We reuse sock_highprio (already bound to 1027) as the egress socket so the source
   port is 1027. linhpsdr wants 512 samples/packet, 16-bit big-endian signed, i.e.
   4-byte BE seq header + 1024-byte payload = 1028-byte datagram; it accumulates a
   16384-sample FFT window at ~10 fps (32 packets/window).
   M2 (this build): the payload is a REAL raw-ADC0 snapshot. On each frame the server
   pulses the WB-arm cfg bit; the FPGA (cores/wb_capture.v) captures WB_BUFFER samples
   into a 32-bit x WB_WORDS dual-port BRAM (two samples/word), which the server reads
   through the axi_hub b00 window at WB_BRAM_BASE and repacks big-endian. Set
   WB_SYNTHETIC 1 to fall back to the M0 two-tone generator (transport-only debug). */
#define WB_SYNTHETIC       0       /* 1 = M0 two-tone generator; 0 = real ADC snapshot */
#define WB_SAMPLES_PER_PKT 512     /* samples per WB datagram (linhpsdr fixed)       */
#define WB_PACKETS         32      /* per 100 ms: 512*32 = 16384 = one FFT window    */
#define WB_BUFFER          (WB_SAMPLES_PER_PKT * WB_PACKETS)  /* 16384 = FFT window  */
#define WB_WORDS           (WB_BUFFER / 2)                    /* 8192 packed 32b words */
#define WB_PERIOD_US       100000  /* 10 fps                                         */
#define WB_PKT_BYTES       (4 + WB_SAMPLES_PER_PKT * 2)   /* 1028                    */
#define WB_BRAM_BASE       0x42000000  /* axi_hub b00 BRAM slot (snapshot readout)   */
#define WB_CAPTURE_US      1000    /* wait after arm for the 131 us snapshot to freeze */

/* ---- FPGA register windows ---- */
volatile uint8_t  *rx_rst;         /* cfg+0 bit0: writer/stream reset (active low) */
volatile uint8_t  *rx_sel;         /* cfg+1: per-DDC ADC select bitmap        */
volatile uint16_t *rx_rate;        /* cfg+2: shared CIC decimation word       */
volatile uint32_t *rx_freq;        /* cfg+4: rx_freq[NUM_DDC] phase increments */
volatile uint32_t *rx_min;         /* cfg+28: DDR ring physical base   (min_addr, cfg[255:224]) */
volatile uint32_t *rx_ring;        /* cfg+32: ring size-1 in 128B bursts       (cfg[287:256]) */
volatile uint8_t  *rx_gpio;        /* cfg+44: open-collector outputs -> E1 exp_p pins (cfg[359:352]) */
volatile uint8_t  *rx_wb;          /* cfg+45 bit0: WB-arm (rising edge starts a snapshot, cfg[360]) */
volatile uint32_t *rx_wptr;        /* sts+0: writer pointer, in 128-byte bursts */
volatile uint8_t  *dma_ram;        /* mmap of the CMA DDR ring (ACP-coherent)  */
volatile uint32_t *wb_bram;        /* mmap of the WB snapshot BRAM (hub b00 @ WB_BRAM_BASE) */

/* Optional per-DDC startup ADC assignment (NUM_DDC args, one per DDC):
     0 = host chooses this DDC's ADC (default),  1 = force ADC0,  2 = force ADC1.
   Forced DDCs ignore the host's ddc-specific ADC bit -- for clients that can't select the
   ADC (CW Skimmer Server, SparkSDR); host-controlled DDCs work as before, so a client that
   can pick the ADC (linhpsdr, Thetis) still steers the DDCs left at 0. */
static uint32_t adc_force_mask = 0;   /* bit d set => DDC d's ADC is forced (host bit ignored) */
static uint32_t adc_force_val  = 0;   /* bit d => forced value ADC1 (only where adc_force_mask set) */

/* ---- sockets ---- */
static int sock_cmd, sock_ddcspec, sock_highprio;
static int sock_data[NUM_DDC];

/* ---- session state ---- */
static uint8_t mac[6];
static volatile int running = 0;
static volatile int have_host = 0;
static struct sockaddr_in host_addr;      /* where data/status are sent (C&C source) */
static uint32_t ddc_enable = 0;
static struct timespec last_cc;

static uint32_t seq_ddc[NUM_DDC];
static uint64_t ts_ddc[NUM_DDC];
static uint32_t seq_status = 0;

/* ---- Diversity (synced DDC pair) state ----
   piHPSDR/Thetis request diversity by enabling ONLY DDC0 (byte 7 = 0x01), pointing DDC1
   at ADC1 (byte 23 = 1), and setting the P2 sync register (DDC-specific byte 1363 bit n =
   "sync DDC n to DDC0"). The radio must then fold the synced DDC's I/Q into DDC0's stream,
   interleaved as (DDC0 I,Q)(DDC1 I,Q)... The FPGA already computes every DDC every cycle
   (both ADCs sit in each 32-byte ring instant), so this is done entirely here: we honor
   the synced DDC's ADC assignment (below) and emit DDC0 as the interleaved DIV stream. */
static volatile int diversity_active  = 0;  /* sync set AND DDC0 enabled                 */
static int          diversity_partner = 1;  /* the DDC folded into DDC0 (bit1 for clients)*/
#define DIV_PAIR_SAMPLES  (SAMPLES_PER_FRAME / 2)   /* 119 interleaved pairs per DIV packet */

/* ---- wideband (WB) state ---- */
static volatile int wb_enable = 0;        /* General byte[23]&1 -> enable ADC0 WB        */
static volatile int wb_ppf = WB_PACKETS;  /* packets per frame (General byte[28]); Thetis
                                             defaults 32, linhpsdr leaves 0 -> WB_PACKETS.
                                             The WB sequence number MUST restart at 0 each
                                             frame and run 0..wb_ppf-1: Thetis's WB receiver
                                             is a per-frame state machine keyed on that
                                             (network.c), while linhpsdr ignores the seq. */
static volatile int wb_rate_ms = WB_PERIOD_US / 1000;  /* inter-frame period (General byte[27],
                                             update rate in ms); Thetis defaults 70, linhpsdr
                                             leaves 0 -> our default. Clamped to [10,1000]. */

/* Convert a client phase word (computed for 122.88 MHz) to the RP NCO phase
   increment at 125 MHz.  Without this the radio tunes ~1.7% high. */
static uint32_t phaseword_to_pinc(uint32_t pw)
{
  return (uint32_t)llround((double)pw * HPSDR_DSP_CLOCK / ADC_CLOCK);
}

static uint32_t be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/* ---------- data path: the FPGA DMAs 32-byte instants into a DDR ring over the
   cache-coherent ACP port; the CPU no longer reads a hardware FIFO. The READER
   thread (core 0) polls the writer pointer and copies whole packets' worth of
   instants out of the DDR ring into a small SPSC ring of raw frames; the SENDER
   thread (core 1) does the per-DDC demux + sendmmsg. The DDR read is cacheable/
   coherent and cheap, so the old two-core FIFO-read wall is gone.
   Single-producer/single-consumer raw-frame ring. */
#define PKT_SIZE    (16 + SAMPLES_PER_FRAME * 6)   /* 1444 bytes on the wire   */
#define RING_LEN    32                             /* raw frames buffered      */
#define SEND_BATCH  16                             /* frames per sendmmsg call */
/* Optional: let the reader demux the first READER_DEMUX channels itself; with the
   DMA read now cheap the sender easily handles all NUM_DDC, so this stays 0. */
#define READER_DEMUX  0

/* ---- DDR ring geometry (MUST match rx.tcl: cfg_data = DMA_RING_BYTES/128 - 1) ---- */
#define CMA_ALLOC         _IOWR('Z', 0, uint32_t)
#define DMA_BURST         128                              /* writer pointer unit      */
#define DMA_RING_BYTES    (768 * 4096)                     /* 3 MiB, 768 pages         */
#define DMA_RING_INSTANTS (DMA_RING_BYTES / FIFO_WORD)     /* 98304 instants (3MiB/32)  */
#define DMA_RING_CFG      (DMA_RING_BYTES / DMA_BURST - 1) /* 24575: writer wrap point */

typedef struct {
  uint32_t enable;                             /* DDC-enable mask for this frame  */
  uint8_t  raw[SAMPLES_PER_FRAME * FIFO_WORD]; /* one packet's worth of instants  */
  uint8_t  pkt[READER_DEMUX < 1 ? 1 : READER_DEMUX][PKT_SIZE]; /* reader-built pkts */
} raw_slot;

static raw_slot ring[RING_LEN];
static volatile uint32_t ring_head = 0;   /* producer index (reader only) */
static volatile uint32_t ring_tail = 0;   /* consumer index (sender only) */
static volatile uint32_t consumer  = 0;   /* DDR-ring read offset in bytes (reader; reset by main) */

/* §7 gain fix (the 4/5 chain runs a constant ~17 dB low) is applied in the FPGA by
   axis_gain between fir_2 and conv_1 — the samples arrive here already at the right
   level, so build_packet just does the 24-bit big-endian gather. */

/* build one DDC packet (header + I/Q gather) from a raw frame into pkt */
static inline void build_packet(uint8_t *pkt, const uint8_t *raw, int ch)
{
  int i;
  uint32_t s = seq_ddc[ch]++;
  pkt[0] = s >> 24; pkt[1] = s >> 16; pkt[2] = s >> 8; pkt[3] = s;
  uint64_t ts = ts_ddc[ch]; ts_ddc[ch] += SAMPLES_PER_FRAME;
  for(i = 0; i < 8; ++i) pkt[4 + i] = (uint8_t)(ts >> (56 - 8 * i));
  pkt[12] = 0; pkt[13] = 24;
  pkt[14] = SAMPLES_PER_FRAME >> 8; pkt[15] = SAMPLES_PER_FRAME & 0xff;
  /* Each raw instant is 8 * 4B LE channels; DDC ch has I at sp[0..3], Q at sp[4..7]
     (26-bit signed in a 32-bit LE word). Emit Q then I, each 24-bit big-endian —
     exactly the byte order the old subset_1 hardware remap produced. */
  const uint8_t *sp = raw + ch * 8;
  uint8_t *dp = pkt + 16;
  for(i = 0; i < SAMPLES_PER_FRAME; ++i)
  {
    dp[0] = sp[6]; dp[1] = sp[5]; dp[2] = sp[4];   /* Q: bits [23:16],[15:8],[7:0] */
    dp[3] = sp[2]; dp[4] = sp[1]; dp[5] = sp[0];   /* I: bits [23:16],[15:8],[7:0] */
    dp += 6; sp += FIFO_WORD;
  }
}

/* gather one instant's I/Q (24-bit big-endian, Q then I) for DDC ch into dp; returns dp+6 */
static inline uint8_t *gather_iq(uint8_t *dp, const uint8_t *sp)
{
  dp[0] = sp[6]; dp[1] = sp[5]; dp[2] = sp[4];   /* Q */
  dp[3] = sp[2]; dp[4] = sp[1]; dp[5] = sp[0];   /* I */
  return dp + 6;
}

/* Build one Diversity packet on DDC0's stream: DIV_PAIR_SAMPLES (119) interleaved pairs
   (DDC0 I,Q)(partner I,Q), so samples-per-frame = 238 (1444-byte packet, MTU-safe). Two
   such packets cover one 238-instant raw frame ('half' 0/1 selects the instant window),
   giving 2x the normal DDC0 packet rate -- exactly what piHPSDR/Thetis expect for DIV.
   Reuses build_packet's byte order/scaling so DIV inherits the proven RX path. */
static inline void build_div_packet(uint8_t *pkt, const uint8_t *raw, int partner, int half)
{
  int i;
  uint32_t s = seq_ddc[0]++;
  pkt[0] = s >> 24; pkt[1] = s >> 16; pkt[2] = s >> 8; pkt[3] = s;
  uint64_t ts = ts_ddc[0]; ts_ddc[0] += DIV_PAIR_SAMPLES;
  for(i = 0; i < 8; ++i) pkt[4 + i] = (uint8_t)(ts >> (56 - 8 * i));
  pkt[12] = 0; pkt[13] = 24;
  pkt[14] = SAMPLES_PER_FRAME >> 8; pkt[15] = SAMPLES_PER_FRAME & 0xff;  /* 238 (119 pairs) */
  const uint8_t *base = raw + (uint32_t)half * DIV_PAIR_SAMPLES * FIFO_WORD;
  uint8_t *dp = pkt + 16;
  for(i = 0; i < DIV_PAIR_SAMPLES; ++i)
  {
    const uint8_t *inst = base + (uint32_t)i * FIFO_WORD;
    dp = gather_iq(dp, inst + 0);            /* DDC0 = ADC0 */
    dp = gather_iq(dp, inst + partner * 8);  /* partner DDC = ADC1 */
  }
}

void *reader_thread(void *arg)
{
  int ch;
  (void)arg;
  const uint32_t nbytes = SAMPLES_PER_FRAME * FIFO_WORD;   /* one packet: 238 * 32 = 7616 */
  while(1)
  {
    if(!running || !have_host) { usleep(1000); continue; }

    /* producer byte offset, from the writer pointer (128-byte bursts) */
    uint32_t prod  = *rx_wptr * DMA_BURST;
    uint32_t avail = (prod + DMA_RING_BYTES - consumer) % DMA_RING_BYTES;

    /* overflow: if the writer has lapped far ahead, drop the backlog and resync the
       consumer to the producer (instant-aligned). Shouldn't trigger in practice. */
    if(avail > DMA_RING_BYTES - DMA_RING_BYTES / 4)
      { consumer = (prod / FIFO_WORD) * FIFO_WORD; continue; }

    if(avail < nbytes) { usleep(100); continue; }   /* < one packet available yet */

    /* backpressure: don't overwrite frames the sender hasn't consumed yet */
    if(ring_head - __atomic_load_n(&ring_tail, __ATOMIC_ACQUIRE) >= RING_LEN)
      { usleep(50); continue; }

    raw_slot *fr = &ring[ring_head % RING_LEN];
    uint32_t en = ddc_enable;
    fr->enable = en;
    /* copy one packet's worth of instants out of the coherent DDR ring, wrapping at
       the ring end (the ring is a whole multiple of the 32-byte instant). */
    {
      uint32_t first = DMA_RING_BYTES - consumer;
      if(first >= nbytes)
        memcpy(fr->raw, (const void *)(dma_ram + consumer), nbytes);
      else
      {
        memcpy(fr->raw, (const void *)(dma_ram + consumer), first);
        memcpy(fr->raw + first, (const void *)dma_ram, nbytes - first);
      }
      consumer += nbytes;
      if(consumer >= DMA_RING_BYTES) consumer -= DMA_RING_BYTES;
    }
    /* demux our share (first READER_DEMUX channels) in the reader's spare time */
    for(ch = 0; ch < READER_DEMUX && ch < NUM_DDC; ++ch)
      if(en & (1u << ch)) build_packet(fr->pkt[ch], fr->raw, ch);
    __atomic_store_n(&ring_head, ring_head + 1, __ATOMIC_RELEASE);
  }
  return NULL;
}

void *sender_thread(void *arg)
{
  static uint8_t txpkt[NUM_DDC][SEND_BATCH][PKT_SIZE];   /* sender-built packets */
  static uint8_t divpkt[2 * SEND_BATCH][PKT_SIZE];       /* Diversity: 2 packets/slot   */
  struct mmsghdr msgs[SEND_BATCH];
  struct iovec   iov[SEND_BATCH];
  struct mmsghdr divmsgs[2 * SEND_BATCH];
  struct iovec   diviov[2 * SEND_BATCH];
  int ch, i;
  uint32_t j, n;
  (void)arg;

  while(1)
  {
    if(!running || !have_host) { usleep(1000); continue; }

    uint32_t head = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE);
    uint32_t avail = head - ring_tail;
    if(avail == 0) { usleep(100); continue; }
    n = avail < SEND_BATCH ? avail : SEND_BATCH;
    uint32_t enable = ring[ring_tail % RING_LEN].enable;
    int div = diversity_active;   /* DDC0 -> interleaved DIV pair; skip its normal build/send */

    /* demux the sender's share (channels >= READER_DEMUX) */
    for(j = 0; j < n; ++j)
    {
      raw_slot *fr = &ring[(ring_tail + j) % RING_LEN];
      for(ch = READER_DEMUX; ch < NUM_DDC; ++ch)
        if((enable & (1u << ch)) && !(div && ch == 0)) build_packet(txpkt[ch][j], fr->raw, ch);
    }
    /* Diversity: DDC0 carries the interleaved (DDC0,partner) pair -- 2 packets per raw
       slot, from source port 1035. Built here and sent instead of DDC0's normal stream. */
    if(div)
    {
      int partner = diversity_partner;
      for(j = 0; j < n; ++j)
      {
        raw_slot *fr = &ring[(ring_tail + j) % RING_LEN];
        build_div_packet(divpkt[2 * j],     fr->raw, partner, 0);
        build_div_packet(divpkt[2 * j + 1], fr->raw, partner, 1);
      }
      for(j = 0; j < 2 * n; ++j)
      {
        diviov[j].iov_base = divpkt[j];
        diviov[j].iov_len  = PKT_SIZE;
        memset(&divmsgs[j], 0, sizeof(divmsgs[j]));
        divmsgs[j].msg_hdr.msg_name    = &host_addr;
        divmsgs[j].msg_hdr.msg_namelen = sizeof(host_addr);
        divmsgs[j].msg_hdr.msg_iov     = &diviov[j];
        divmsgs[j].msg_hdr.msg_iovlen  = 1;
      }
      sendmmsg(sock_data[0], divmsgs, 2 * n, 0);
    }

    /* one sendmmsg per enabled DDC: reader-built packets live in the ring slot,
       sender-built ones in txpkt (DDC0 is handled by the DIV path above when active) */
    for(ch = 0; ch < NUM_DDC; ++ch)
    {
      if(!(enable & (1u << ch))) continue;
      if(div && ch == 0) continue;
      for(j = 0; j < n; ++j)
      {
        raw_slot *fr = &ring[(ring_tail + j) % RING_LEN];
        iov[j].iov_base = (ch < READER_DEMUX) ? fr->pkt[ch] : txpkt[ch][j];
        iov[j].iov_len  = PKT_SIZE;
        memset(&msgs[j], 0, sizeof(msgs[j]));
        msgs[j].msg_hdr.msg_name    = &host_addr;
        msgs[j].msg_hdr.msg_namelen = sizeof(host_addr);
        msgs[j].msg_hdr.msg_iov     = &iov[j];
        msgs[j].msg_hdr.msg_iovlen  = 1;
      }
      sendmmsg(sock_data[ch], msgs, n, 0);
    }
    __atomic_store_n(&ring_tail, ring_tail + n, __ATOMIC_RELEASE);
  }
  return NULL;
}

/* ---------- high-priority status to the host (port 1025), ~50 ms ---------- */
void *status_thread(void *arg)
{
  uint8_t st[60];
  while(1)
  {
    usleep(50000);
    if(!have_host) continue;

    memset(st, 0, sizeof(st));
    uint32_t s = seq_status++;
    st[0] = s >> 24; st[1] = s >> 16; st[2] = s >> 8; st[3] = s;
    st[4] = 0x10;                 /* bit4: PLL locked (we have no external 10MHz) */
    /* byte 5 ADC overload, FIFO depths, etc. left 0 */
    sendto(sock_ddcspec, st, sizeof(st), 0,
           (struct sockaddr *)&host_addr, sizeof(host_addr));
  }
  return NULL;
}

/* Fill buf[WB_BUFFER] with one FFT window of int16 samples.
   WB_SYNTHETIC: two phase-continuous tones at fs/8 (15.625 MHz) and fs/4 (31.25 MHz),
   landing on exact bins 2048/4096 of linhpsdr's 16384-pt window -> two clean peaks at
   1/4 and 1/2 of the 0-62.5 MHz span (transport-only debug).
   Otherwise: arm the FPGA snapshot, wait for the 131 us capture to freeze, then read
   WB_WORDS packed 32-bit words from the BRAM (low half = first/even sample, high half =
   second/odd sample, both signed 16-bit). */
static void wb_fill_window(int16_t *buf)
{
#if WB_SYNTHETIC
  static uint64_t wb_n = 0;
  const double w1 = 2.0 * M_PI / 8.0, w2 = 2.0 * M_PI / 4.0;
  int i;
  for(i = 0; i < WB_BUFFER; ++i)
  {
    double t = (double)(wb_n++);
    long v = lround(8000.0 * sin(w1 * t) + 6000.0 * sin(w2 * t));
    if(v > 32767) v = 32767; else if(v < -32768) v = -32768;
    buf[i] = (int16_t)v;
  }
#else
  int i;
  *rx_wb = 1;                 /* rising edge -> FPGA captures WB_BUFFER samples (~131 us) */
  usleep(WB_CAPTURE_US);      /* wait for the snapshot to complete and freeze            */
  for(i = 0; i < WB_WORDS; ++i)
  {
    uint32_t w = wb_bram[i];
    buf[2 * i]     = (int16_t)(w & 0xffff);          /* even (first) sample  */
    buf[2 * i + 1] = (int16_t)((w >> 16) & 0xffff);  /* odd  (second) sample */
  }
  *rx_wb = 0;                 /* lower arm to re-ready the gate for the next snapshot */
#endif
}

/* ---------- wideband (WB) sender to the host (source port 1027), ~10 fps ----------
   Emits one frame of `ppf` datagrams (WB_SAMPLES_PER_PKT samples each) per WB_PERIOD_US,
   only while a host is present and has enabled WB (General byte[23]&1). Each sample is
   16-bit big-endian; egress is from sock_highprio (bound to 1027) so the UDP source port
   is 1027. The datagram sequence number RESTARTS AT 0 each frame and runs 0..ppf-1 --
   Thetis's WB receiver is a per-frame state machine that waits for seq 0 then expects
   1..ppf-1 (ChannelMaster/network.c); linhpsdr ignores the seq and just concatenates, so
   per-frame numbering satisfies both. `ppf` (<= WB_PACKETS) comes from General byte[28];
   at ppf < WB_PACKETS we send the first ppf*512 samples of the 16384-sample capture. */
void *wb_thread(void *arg)
{
  static int16_t win[WB_BUFFER];
  uint8_t pkt[WB_PKT_BYTES];
  int p, k, idx;
  (void)arg;

  while(1)
  {
    if(!have_host || !wb_enable) { usleep(2000); continue; }

    int ppf = wb_ppf;                        /* snapshot the frame size for this frame */
    if(ppf < 1 || ppf > WB_PACKETS) ppf = WB_PACKETS;
    wb_fill_window(win);
    idx = 0;
    for(p = 0; p < ppf; ++p)
    {
      uint32_t s = (uint32_t)p;              /* per-frame sequence 0..ppf-1 */
      uint8_t *dp = pkt + 4;
      pkt[0] = s >> 24; pkt[1] = s >> 16; pkt[2] = s >> 8; pkt[3] = s;
      for(k = 0; k < WB_SAMPLES_PER_PKT; ++k)
      {
        int16_t v = win[idx++];
        dp[0] = (uint8_t)((v >> 8) & 0xff);   /* 16-bit big-endian signed */
        dp[1] = (uint8_t)(v & 0xff);
        dp += 2;
      }
      sendto(sock_highprio, pkt, sizeof(pkt), 0,
             (struct sockaddr *)&host_addr, sizeof(host_addr));
    }
    usleep(wb_rate_ms * 1000);   /* honor the host's WB update rate (General byte[27]) */
  }
  return NULL;
}

//
//
// The microphone thread just sends silence, that is
// a "zeroed" mic frame every 1.333 msec and needs to
// be sent for some app's timing purposes.
//
void *mic_thread(void *data)
{
    int sock;
    unsigned long seqnum = 0; 
    struct sockaddr_in addr;
    unsigned char mic_buffer[132];
    unsigned char *p;
    int yes = 1; 
    struct timespec delay;
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) { 
        perror("***** ERROR: Mic thread: socket");
        return NULL;
    }    

    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&yes, sizeof(yes));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, (void *)&yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(PORT_MIC);

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { 
        perror("mic_thread ERROR: bind");
        close(sock);
        return NULL;
    }    

    memset(mic_buffer, 0, 132);
    clock_gettime(CLOCK_MONOTONIC, &delay);

    while (1) {
        // Idle politely when not streaming or before a client connects, and reset the
        // absolute timer base each idle tick so we don't fire a backlog burst of frames
        // on resume (matches the reader/sender idle pattern).
        if (!running || !have_host) {
            usleep(1000);
            seqnum = 0;
            clock_gettime(CLOCK_MONOTONIC, &delay);
            continue;
        }

        // update seq number
        p = mic_buffer;
        *(uint32_t*)p = htonl(seqnum++);
        p += 4;

        // 64 samples with 48000 kHz, makes 1333333 nsec
        delay.tv_nsec += 1333333;

        while (delay.tv_nsec >= 1000000000) {
            delay.tv_nsec -= 1000000000;
            delay.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &delay, NULL);

        if (sendto(sock, mic_buffer, 132, 0, (struct sockaddr * )&host_addr, sizeof(host_addr)) < 0) {
            perror("***** ERROR: Mic thread sendto");
            break;
        }
    }

    close(sock);
    return NULL;
}

/* ---------- discovery reply (from port 1024) ---------- */
static void send_discovery_reply(struct sockaddr_in *to, socklen_t tolen)
{
  uint8_t reply[60];
  memset(reply, 0, sizeof(reply));
  /* bytes 0-3 sequence = 0 */
  reply[4]  = running ? 3 : 2;   /* 2 = available, 3 = in use */
  memcpy(reply + 5, mac, 6);     /* bytes 5-10 board MAC */
  reply[11] = BOARD_TYPE;        /* Angelia */
  reply[12] = PROTOCOL_VERSION;
  reply[13] = CODE_VERSION;
  reply[20] = NUM_DDC;           /* # DDCs -> client's receiver count */
  reply[21] = 1;                 /* frequency sent as phase word */
  reply[22] = 0;                 /* big-endian, 3-byte I/Q */
  sendto(sock_cmd, reply, sizeof(reply), 0, (struct sockaddr *)to, tolen);
}

/* ---------- DDC-specific (port 1025): ADC assignment + sample rate ---------- */
static void process_ddc_specific(const uint8_t *b, ssize_t n)
{
  int ch, adc, rate_khz;
  uint8_t sel = 0;
  uint16_t rate = *rx_rate;

  ddc_enable = b[7];             /* DDC-enable bitmap (DDC0..7) */

  /* Diversity sync register (byte 1363): bit n => DDC n is synced into DDC0. Only present
     in a full-length DDC-specific packet; a short one implies no sync. Synced DDCs are NOT
     in the enable bitmap, so fold them in below so their ADC assignment is still applied. */
  uint8_t sync = (n > 1363) ? b[1363] : 0;
  uint8_t active = (uint8_t)(ddc_enable | sync);

  for(ch = 0; ch < NUM_DDC; ++ch)
  {
    if(!(active & (1u << ch))) continue;
    adc      = b[17 + ch * 6];                        /* 0 = ADC0, 1 = ADC1 */
    rate_khz = (b[18 + ch * 6] << 8) | b[19 + ch * 6];
    sel |= (adc & 1) << ch;
    if(rate_khz > 0) rate = (uint16_t)(RATE_BASE / rate_khz);  /* shared: last enabled wins */
  }

  /* apply host ADC bits only to host-controlled DDCs; pinned DDCs (adc_force_mask) keep their ADC */
  *rx_sel  = (uint8_t)((sel & ~adc_force_mask) | (adc_force_val & adc_force_mask));
  /* Diversity is active when a DDC is synced to DDC0 and DDC0 itself is enabled: DDC0's
     stream then carries the interleaved (DDC0,partner) pair (see build_div_packet). */
  if(sync & ~1u)                                        /* some DDC (>0) synced to DDC0 */
    for(ch = 1; ch < NUM_DDC; ++ch) { if(sync & (1u << ch)) { diversity_partner = ch; break; } }
  diversity_active = (sync & ~1u) && (ddc_enable & 1u);
  *rx_rate = rate;               /* 48k->1000, 96k->500, 192k->250 (384k->125, Phase 3) */
}

/* ---------- high-priority (port 1027): run bit + per-DDC phase words ---------- */
static void process_high_priority(const uint8_t *b, ssize_t n)
{
  int ch;
  int newrun = b[4] & 1;
  for(ch = 0; ch < NUM_DDC; ++ch)
    rx_freq[ch] = phaseword_to_pinc(be32(b + 9 + ch * 4));

  /* Open-collector outputs for band/filter (BCD) control: P2 High-Priority byte
     1401 holds OC1..OC7 (bit1..bit7). Drive OC1-4 onto the E1 expansion pins
     DIO4_P-DIO7_P exactly as the Protocol-1 transceiver did ((x & 0x1e) << 3),
     so filter boards wired for P1 keep working under P2. Only apply when the
     client sent a full-length high-priority packet. */
  if(n >= 1402) *rx_gpio = (uint8_t)((b[1401] & 0x1e) << 3);

  if(newrun && !running)   /* run-start edge: restart the DMA writer + reset stream/ring state */
  {
    *rx_rst &= ~1; *rx_rst |= 1;        /* re-zero the writer pointer; ring refills from base */
    consumer = 0;
    for(ch = 0; ch < NUM_DDC; ++ch) { seq_ddc[ch] = 0; ts_ddc[ch] = 0; }
    ring_head = 0; ring_tail = 0;
    __sync_synchronize();
  }
  running = newrun;
  clock_gettime(CLOCK_MONOTONIC, &last_cc);
}

/* Hold the DMA writer in reset before we exit, so it stops touching the CMA ring
   BEFORE the kernel frees it. Otherwise the FPGA keeps DMAing into the freed DDR
   and corrupts whatever the kernel reallocates there. Covers SIGTERM/SIGINT, so
   swap the server with `pkill -TERM -f`, not -9. */
static void on_signal(int sig)
{
  (void)sig;
  if(rx_rst) *rx_rst &= ~1;
  _exit(0);
}

int main(int argc, char *argv[])
{
  int fd, i, yes = 1;
  volatile void *cfg, *sts;
  struct ifreq hwaddr;
  struct sockaddr_in addr;
  pthread_t tid;

  /* Optional per-DDC ADC assignment: NUM_DDC args, each:
       0 = host chooses (default),  1 = force ADC0,  2 = force ADC1.
     Forced DDCs ignore the host's ddc-specific ADC bit (for clients that can't select the
     ADC, e.g. CW Skimmer Server / SparkSDR); 0 leaves the DDC host-controlled as before. */
  if(argc > 1)
  {
    if(argc != 1 + NUM_DDC)
    {
      fprintf(stderr, "Usage: %s [<adc0> ... <adc%d>]   (%d values, each 0=host/1=ADC0/2=ADC1)\n",
              argv[0], NUM_DDC - 1, NUM_DDC);
      return EXIT_FAILURE;
    }
    for(i = 0; i < NUM_DDC; ++i)
    {
      char *end;
      long v;
      errno = 0;
      v = strtol(argv[i + 1], &end, 10);
      if(errno != 0 || end == argv[i + 1] || v < 0 || v > 2)
      {
        fprintf(stderr, "Usage: %s [<adc0> ... <adc%d>]   (%d values, each 0=host/1=ADC0/2=ADC1)\n",
                argv[0], NUM_DDC - 1, NUM_DDC);
        return EXIT_FAILURE;
      }
      if(v != 0)                                   /* 1 or 2 -> pin this DDC */
      {
        adc_force_mask |= (uint32_t)1 << i;
        adc_force_val  |= (uint32_t)(v - 1) << i;  /* 1 -> ADC0 (0), 2 -> ADC1 (1) */
      }
    }
  }

  if((fd = open("/dev/mem", O_RDWR)) < 0) { perror("open /dev/mem"); return EXIT_FAILURE; }
  cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x40000000);
  sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x41000000);
  /* WB snapshot BRAM (axi_hub b00): WB_WORDS * 4 = 32 KiB, page-rounded */
  wb_bram = (uint32_t *)mmap(NULL, (WB_WORDS * 4 + sysconf(_SC_PAGESIZE) - 1)
                             & ~(sysconf(_SC_PAGESIZE) - 1),
                             PROT_READ | PROT_WRITE, MAP_SHARED, fd, WB_BRAM_BASE);
  close(fd);

  rx_rst  = (uint8_t  *)(cfg + 0);
  rx_sel  = (uint8_t  *)(cfg + 1);
  rx_rate = (uint16_t *)(cfg + 2);
  rx_freq = (uint32_t *)(cfg + 4);
  rx_min  = (uint32_t *)(cfg + 28);
  rx_ring = (uint32_t *)(cfg + 32);
  rx_gpio = (uint8_t  *)(cfg + 44);
  rx_wb   = (uint8_t  *)(cfg + 45);
  rx_wptr = (uint32_t *)(sts + 0);

  /* stop the writer cleanly on a graceful kill (see on_signal) */
  signal(SIGTERM, on_signal);
  signal(SIGINT,  on_signal);

  /* allocate the contiguous DDR ring and point the DMA writer at it */
  {
    int cfd;
    uint32_t sz = DMA_RING_BYTES;
    if((cfd = open("/dev/cma", O_RDWR)) < 0) { perror("open /dev/cma"); return EXIT_FAILURE; }
    if(ioctl(cfd, CMA_ALLOC, &sz) < 0) { perror("CMA_ALLOC"); return EXIT_FAILURE; }
    /* the ioctl overwrites sz with the physical base address of the allocation */
    dma_ram = mmap(NULL, DMA_RING_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, 0);
    if(dma_ram == MAP_FAILED) { perror("mmap cma"); return EXIT_FAILURE; }
    /* keep cfd open for the process lifetime so the buffer stays allocated */
    *rx_min  = sz;
    *rx_ring = DMA_RING_CFG;
  }

  /* sensible defaults */
  *rx_rate = RATE_BASE / 48;                /* 48 ksps default */
  *rx_sel  = adc_force_val & adc_force_mask;   /* pinned DDCs -> their ADC; rest ADC0 until host sets */
  *rx_gpio = 0;                             /* open-collector / filter pins low */
  *rx_wb   = 0;                             /* WB-arm low (gate idle until wb_thread pulses) */
  for(i = 0; i < NUM_DDC; ++i)
    rx_freq[i] = phaseword_to_pinc((uint32_t)floor(600000.0 / HPSDR_DSP_CLOCK * 4294967296.0 + 0.5));

  /* start the DMA writer streaming into the ring */
  *rx_rst &= ~1; *rx_rst |= 1;

  /* read board MAC from eth0 */
  {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&hwaddr, 0, sizeof(hwaddr));
    strncpy(hwaddr.ifr_name, "eth0", IFNAMSIZ - 1);
    ioctl(s, SIOCGIFHWADDR, &hwaddr);
    for(i = 0; i < 6; ++i) mac[i] = hwaddr.ifr_addr.sa_data[i];
    close(s);
  }

  /* bind the three inbound command sockets */
  sock_cmd      = socket(AF_INET, SOCK_DGRAM, 0);
  sock_ddcspec  = socket(AF_INET, SOCK_DGRAM, 0);
  sock_highprio = socket(AF_INET, SOCK_DGRAM, 0);
  int inports[3] = { PORT_COMMAND, PORT_DDC_SPECIFIC, PORT_HIGH_PRIORITY };
  int insocks[3] = { sock_cmd, sock_ddcspec, sock_highprio };
  for(i = 0; i < 3; ++i)
  {
    setsockopt(insocks[i], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(inports[i]);
    if(bind(insocks[i], (struct sockaddr *)&addr, sizeof(addr)) < 0)
    { perror("bind command socket"); return EXIT_FAILURE; }
  }

  /* one outbound socket per DDC, bound to source port 1035+n */
  for(i = 0; i < NUM_DDC; ++i)
  {
    sock_data[i] = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(sock_data[i], SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(PORT_DDC_DATA0 + i);
    if(bind(sock_data[i], (struct sockaddr *)&addr, sizeof(addr)) < 0)
    { perror("bind data socket"); return EXIT_FAILURE; }
  }

  /* Silence ICMP port-unreachable on the P2 host->radio ports this RX-only receiver
     does not service: 1028 (speaker/LR audio) and 1029.. (DUC / TX I&Q). Thetis streams
     these continuously even in pure RX. If the ports are unbound the board's kernel answers
     each datagram with ICMP port-unreachable; on Windows that latches WSAECONNRESET onto
     Thetis's shared RX socket and freezes its receive loop (sooner at higher sample rates;
     piHPSDR on Linux is immune). Bind throwaway sockets (never read) so the datagrams land
     silently and no ICMP is emitted. */
  for(i = PORT_HIGH_PRIORITY + 1; i < PORT_DDC_DATA0; ++i)   /* 1028..1034 */
  {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(i);
    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      perror("bind sink socket");   /* non-fatal: worst case a stray ICMP */
  }

  /* Give the reader core 0 ALL to itself (the latency-bound volatile FIFO read is
     the throughput limiter); put the sender, status thread and this command/main
     thread together on core 1. Without isolating the reader, main/status float onto
     core 0 and steal the ~10% headroom needed for 6 DDC x 768 ksps. */
  {
    pthread_t rtid, stid, sttid, mtid, wtid;
    cpu_set_t cs;
    pthread_create(&rtid,  NULL, reader_thread, NULL);
    pthread_create(&stid,  NULL, sender_thread, NULL);
    pthread_create(&sttid, NULL, status_thread, NULL);
    pthread_create(&mtid,  NULL, mic_thread, NULL);
    pthread_create(&wtid,  NULL, wb_thread, NULL);
    CPU_ZERO(&cs); CPU_SET(0, &cs); pthread_setaffinity_np(rtid, sizeof(cs), &cs);
    CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(stid,  sizeof(cs), &cs);
    pthread_setaffinity_np(sttid, sizeof(cs), &cs);
    pthread_setaffinity_np(mtid, sizeof(cs), &cs);
    pthread_setaffinity_np(wtid, sizeof(cs), &cs);
    sched_setaffinity(0, sizeof(cs), &cs);      /* main (command) thread -> core 1 */
    pthread_detach(rtid); pthread_detach(stid); pthread_detach(sttid); pthread_detach(mtid);
    pthread_detach(wtid);
  }

  clock_gettime(CLOCK_MONOTONIC, &last_cc);

  while(1)
  {
    fd_set fds;
    struct timeval tv = { 0, 200000 };   /* 200 ms so the watchdog runs */
    int maxfd = sock_cmd;
    FD_ZERO(&fds);
    FD_SET(sock_cmd, &fds);      FD_SET(sock_ddcspec, &fds);  FD_SET(sock_highprio, &fds);
    if(sock_ddcspec > maxfd)  maxfd = sock_ddcspec;
    if(sock_highprio > maxfd) maxfd = sock_highprio;

    if(select(maxfd + 1, &fds, NULL, NULL, &tv) > 0)
    {
      uint8_t buffer[2048];
      struct sockaddr_in from;
      socklen_t fromlen = sizeof(from);
      ssize_t n;

      if(FD_ISSET(sock_cmd, &fds))
      {
        n = recvfrom(sock_cmd, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &fromlen);
        if(n >= 5 && !(buffer[0] == 0xEF && buffer[1] == 0xFE))
        {
          /* SparkSDR/piHPSDR also broadcast a Protocol-1 (Metis) discovery (starts with the
             0xEFFE magic) alongside the Protocol-2 one. Its byte[4] is 0x00, which would
             otherwise be taken as a P2 "general" packet and re-point this receiver's stream to
             the discovery socket's port, freezing an in-progress DDC. Skip any P1 packet
             (the guard above) on this P2-only receiver. */
          if(buffer[4] == 0x02)          /* discovery (from the ephemeral discovery socket) */
          {
            send_discovery_reply(&from, fromlen);
          }
          else if(buffer[4] == 0x00)     /* general packet: this is the session's C&C source */
          {
            host_addr = from; have_host = 1;
            clock_gettime(CLOCK_MONOTONIC, &last_cc);
            /* byte[23] bit0 = enable wideband display for ADC0 (see wb_thread).
               Thetis also sets byte[24:25]=samples/packet (512), [26]=sample size (16),
               [27]=update rate ms, [28]=packets/frame (32); linhpsdr leaves 24..28 = 0.
               We honor packets/frame (clamped to our fixed 512-sample, 16384-capture
               geometry) and update rate; out-of-range/zero falls back to our defaults. */
            if(n >= 24) wb_enable = buffer[23] & 1;
            if(n >= 29)
            {
              int ppf = buffer[28];
              int ur  = buffer[27];
              wb_ppf     = (ppf >= 1 && ppf <= WB_PACKETS) ? ppf : WB_PACKETS;
              wb_rate_ms = (ur >= 10 && ur <= 1000) ? ur : (WB_PERIOD_US / 1000);
            }
          }
        }
      }
      if(FD_ISSET(sock_ddcspec, &fds))
      {
        n = recvfrom(sock_ddcspec, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &fromlen);
        if(n >= 23) { host_addr = from; have_host = 1; clock_gettime(CLOCK_MONOTONIC, &last_cc); process_ddc_specific(buffer, n); }
      }
      if(FD_ISSET(sock_highprio, &fds))
      {
        n = recvfrom(sock_highprio, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &fromlen);
        if(n >= 13) { host_addr = from; have_host = 1; process_high_priority(buffer, n); }
      }
    }

    /* session watchdog: if no C&C for >1 s, drop RUN and forget the host, so we stop
       streaming data AND 50 ms status packets to a stale address. Otherwise those
       status packets flood whoever later reuses that UDP port (e.g. a client's
       discovery socket), whose recvfrom then never times out -> discovery hangs. */
    if(have_host)
    {
      struct timespec t;
      clock_gettime(CLOCK_MONOTONIC, &t);
      if((t.tv_sec - last_cc.tv_sec) + (t.tv_nsec - last_cc.tv_nsec) * 1e-9 > 1.0)
      {
        running = 0; have_host = 0;
      }
    }
  }

  return EXIT_SUCCESS;
}
