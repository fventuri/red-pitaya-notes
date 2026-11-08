/*
 * sdr-receiver-hpsdr2 - openHPSDR Ethernet Protocol 2 receiver for the
 * Red Pitaya / TRX-duo (Zynq 7010).  Narrow (48-ksps) build.
 *
 * The FPGA implements NUM_DDC (16) DDCs. Standard HPSDR clients cap the number of
 * receivers per radio (linhpsdr 8, Thetis 12), so instead of advertising one 16-DDC
 * radio (which needs a patched client) this server presents the DDCs as NUM_RADIOS
 * virtual HPSDR radios of DDC_PER_RADIO each. Each virtual radio lives on its own
 * network interface (eth0 and a macvlan mvl0 created in start.sh) with that interface's
 * own MAC/IP, so a stock linhpsdr/piHPSDR/Thetis discovers two independent 8-DDC radios.
 *
 * Multi-homing: eth0 and mvl0 share one IP subnet, so the sockets are shared (one per
 * UDP port, bound to INADDR_ANY) and IP_PKTINFO is used to (a) route each received packet
 * to the radio whose interface it arrived on and (b) pin every reply/stream to the right
 * egress interface + source IP. Radio r owns physical DDCs [r*DDC_PER_RADIO ..
 * r*DDC_PER_RADIO + DDC_PER_RADIO-1], presented to its client as local DDC 0..N-1.
 *
 * One process owns the single FPGA DMA ring (all NUM_DDC DDCs). A shared reader copies
 * instants out of the coherent DDR ring; a shared sender demuxes each radio's slice of
 * DDCs to that radio's client. Per-DDC ADC selection is preserved (each radio drives its
 * own bits of the shared rx_sel register).
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

/* ---- configuration ----
 * The FPGA has NUM_DDC DDCs (== n_ddc in rx.tcl). They are presented as NUM_RADIOS
 * virtual radios of DDC_PER_RADIO each; NUM_RADIOS*DDC_PER_RADIO MUST equal NUM_DDC.
 * DDC_PER_RADIO <= 8 keeps every stock client happy. This narrow receiver is 48-ksps
 * only: the FPGA has no programmable-rate register, so the server never writes a rate
 * word - it just forces/advertises 48 ksps. */
#define NUM_RADIOS         2       /* virtual HPSDR radios (one per network interface) */
#define DDC_PER_RADIO      8       /* DDCs each virtual radio presents (<=8 for stock clients) */
#define NUM_DDC            (NUM_RADIOS * DDC_PER_RADIO)  /* == rx.tcl n_ddc (16) */
#define BOARD_TYPE         3       /* linhpsdr device enum: Angelia (2 ADCs) */
#define CODE_VERSION       1       /* firmware/code version (discovery [13]) */
#define PROTOCOL_VERSION   39      /* openHPSDR protocol version *10 ([12])  */
#define SAMPLES_PER_FRAME  238     /* 24-bit I/Q pairs per DDC packet        */
#define FIFO_WORD          (NUM_DDC * 8) /* bytes per instant in the DDR ring:
                                      NUM_DDC * (I,Q) * 4B LE, each a 26-bit signed
                                      value in a 32-bit LE word */
#define EN_BYTES           ((DDC_PER_RADIO + 7) / 8) /* enable mask bytes per radio */

#define ADC_CLOCK          125000000.0
#define HPSDR_DSP_CLOCK    122880000.0    /* clients hardcode this           */

/* ---- Protocol-2 UDP ports (same on every radio; disambiguated by interface via
   IP_PKTINFO on receive and pinned egress on transmit) ---- */
#define PORT_COMMAND       1024    /* discovery/general/command in; reply out */
#define PORT_DDC_SPECIFIC  1025    /* DDC-specific in; high-priority status out */
#define PORT_MIC           1026
#define PORT_HIGH_PRIORITY 1027    /* high-priority in (run + phase words)     */
#define PORT_DDC_DATA0     1035    /* DDC n I/Q out from source port 1035+n    */

/* ---- FPGA register windows ---- (cfg byte map derived from NUM_DDC, see rx.tcl)
 *   cfg+0            bit0 : writer/stream reset (active low)
 *   cfg+4 ..         freq[0..NUM_DDC-1], 32-bit phase increment each
 *   OFF_SEL          per-DDC ADC-select bitmap (NUM_DDC bits)
 *   OFF_MIN          DDR ring physical base (min_addr)
 *   OFF_RING         ring size-1 in 128-byte bursts
 *   OFF_GPIO         open-collector outputs -> E1 exp_p pins
 * where OFF_TAIL = 4 + 4*NUM_DDC. The programmable-rate word is gone (48-only). */
#define OFF_TAIL   (4 + 4 * NUM_DDC)
#define OFF_SEL    (OFF_TAIL)
#define OFF_MIN    (OFF_TAIL + 4)
#define OFF_RING   (OFF_TAIL + 8)
#define OFF_GPIO   (OFF_TAIL + 12)

volatile uint8_t  *rx_rst;         /* cfg+0 bit0: writer/stream reset (active low) */
volatile uint32_t *rx_sel;         /* OFF_SEL: per-DDC ADC select bitmap       */
volatile uint32_t *rx_freq;        /* cfg+4: rx_freq[NUM_DDC] phase increments */
volatile uint32_t *rx_min;         /* OFF_MIN: DDR ring physical base           */
volatile uint32_t *rx_ring;        /* OFF_RING: ring size-1 in 128B bursts      */
volatile uint8_t  *rx_gpio;        /* OFF_GPIO: open-collector outputs -> E1 exp_p pins */
volatile uint32_t *rx_wptr;        /* sts+0: writer pointer, in 128-byte bursts */
volatile uint8_t  *dma_ram;        /* mmap of the CMA DDR ring (ACP-coherent)  */

/* Optional per-DDC startup ADC assignment (NUM_DDC args, one per physical DDC):
     0 = host chooses this DDC's ADC (default),  1 = force ADC0,  2 = force ADC1.
   Forced DDCs ignore the host's ddc-specific ADC bit -- for clients that can't select the
   ADC (CW Skimmer Server, SparkSDR); host-controlled DDCs work as before, so a client that
   can pick the ADC (linhpsdr, Thetis) still steers the DDCs left at 0. */
static uint32_t adc_force_mask = 0;   /* bit d set => DDC d's ADC is forced (host bit ignored) */
static uint32_t adc_force_val  = 0;   /* bit d => forced value ADC1 (only where adc_force_mask set) */

/* ---- shared sockets (one per UDP port; multi-homed via IP_PKTINFO) ---- */
static int sock_cmd, sock_ddcspec, sock_highprio, sock_mic;
static int sock_data[DDC_PER_RADIO];

/* control-message buffer big enough for one IP_PKTINFO ancillary object, aligned */
typedef union { char buf[CMSG_SPACE(sizeof(struct in_pktinfo))]; struct cmsghdr align; } cmsgbuf_t;

/* Convert a client phase word (computed for 122.88 MHz) to the RP NCO phase
   increment at 125 MHz.  Without this the radio tunes ~1.7% high. */
static uint32_t phaseword_to_pinc(uint32_t pw)
{
  return (uint32_t)llround((double)pw * HPSDR_DSP_CLOCK / ADC_CLOCK);
}

/* A DDC whose NCO phase increment is 0 has a STATIC NCO (cos = full-scale constant,
   sin = 0), so its mixer output is a large constant DC. The 4-stage CIC's ~10^12 DC gain
   drives that channel near overflow, and in the time-shared cic_ts_bank125 it couples into
   the OTHER (tuned) DDCs -> an on-air DC spike + ~1.3 kHz comb across every tuned channel.
   HPSDR clients leave DDCs they aren't using at phase word 0, so we must never let any
   rx_freq[ch] sit at 0. Park such a DDC at a harmless, distinct, out-of-passband tone (the
   48 ksps FIR removes it) so its NCO rotates and produces ~0 baseband DC = no leakage.
   (Measured: all-DDCs-tuned is clean; parking commanded-0 DDCs reproduces that clean state.) */
static uint32_t park_pinc(int ch)
{
  double f = 600000.0 + 12500.0 * ch;   /* 600.0 .. 787.5 kHz for ch 0..15, all out of band */
  return phaseword_to_pinc((uint32_t)floor(f / HPSDR_DSP_CLOCK * 4294967296.0 + 0.5));
}

static uint32_t be32(const uint8_t *p)
{
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

/* ---------- data path: the FPGA DMAs FIFO_WORD-byte instants into a DDR ring over the
   cache-coherent ACP port. The READER thread polls the writer pointer and copies whole
   packets' worth of instants (all NUM_DDC DDCs) out of the DDR ring into a small SPSC ring
   of raw frames; the SENDER thread demuxes each radio's DDC slice and sendmmsg's it to that
   radio's client. The DDR read is cacheable/coherent and cheap.
   Single-producer/single-consumer raw-frame ring. */
#define PKT_SIZE    (16 + SAMPLES_PER_FRAME * 6)   /* 1444 bytes on the wire   */
#define RING_LEN    32                             /* raw frames buffered      */
#define SEND_BATCH  16                             /* frames per sendmmsg call */

/* ---- DDR ring geometry (MUST match rx.tcl: cfg_data = DMA_RING_BYTES/128 - 1) ---- */
#define CMA_ALLOC         _IOWR('Z', 0, uint32_t)
#define DMA_BURST         128                              /* writer pointer unit      */
#define DMA_RING_BYTES    (768 * 4096)                     /* 3 MiB, 768 pages         */
#define DMA_RING_INSTANTS (DMA_RING_BYTES / FIFO_WORD)     /* 49152 instants (64B word) */
#define DMA_RING_CFG      (DMA_RING_BYTES / DMA_BURST - 1) /* 24575: writer wrap point */

typedef struct {
  uint8_t raw[SAMPLES_PER_FRAME * FIFO_WORD]; /* one packet's worth of instants (all DDCs) */
} raw_slot;

static raw_slot ring[RING_LEN];
static volatile uint32_t ring_head = 0;   /* producer index (reader only) */
static volatile uint32_t ring_tail = 0;   /* consumer index (sender only) */
static volatile uint32_t consumer  = 0;   /* DDR-ring read offset in bytes (reader only) */

/* ---- one virtual HPSDR radio: its interface, its slice of the DDCs, its client ---- */
typedef struct {
  int              index;                    /* 0..NUM_RADIOS-1                        */
  const char      *ifname;                   /* "eth0", "mvl0", ...                    */
  int              ddc_base;                  /* physical DDC index of this radio's DDC0 */
  unsigned         ifindex;                  /* if_nametoindex(ifname) - RX routing     */
  struct in_addr   ip;                       /* this interface's IPv4 - TX source addr  */
  uint8_t          mac[6];                    /* this interface's MAC (advertised)      */
  cmsgbuf_t        cmsg;                       /* prebuilt IP_PKTINFO for this radio's TX */

  volatile int     running, have_host;
  struct sockaddr_in host_addr;              /* this radio's client (C&C source)       */
  uint32_t         ddc_enable;               /* local enable mask (DDC_PER_RADIO bits) */
  struct timespec  last_cc;

  uint32_t         seq_ddc[DDC_PER_RADIO];
  uint64_t         ts_ddc[DDC_PER_RADIO];
  uint32_t         seq_status;
} radio_t;

static radio_t radios[NUM_RADIOS];
static const char *ifnames[NUM_RADIOS] = { "eth0", "mvl0" };

static int any_active(void)
{
  int i;
  for(i = 0; i < NUM_RADIOS; ++i)
    if(radios[i].running && radios[i].have_host) return 1;
  return 0;
}

static radio_t *radio_by_ifindex(unsigned ifindex)
{
  int i;
  for(i = 0; i < NUM_RADIOS; ++i)
    if(radios[i].ifindex == ifindex) return &radios[i];
  return NULL;
}

/* Prebuild radio r's IP_PKTINFO control block: outgoing packets carry this so they egress
   r's interface with r's IP as the source address (correct on a multi-homed same-subnet
   host, where plain source-address selection would pick either interface's IP). */
static void build_pktinfo(radio_t *r)
{
  struct msghdr m;
  struct cmsghdr *c;
  struct in_pktinfo *pi;
  memset(&r->cmsg, 0, sizeof(r->cmsg));
  memset(&m, 0, sizeof(m));
  m.msg_control = r->cmsg.buf;
  m.msg_controllen = sizeof(r->cmsg.buf);
  c = CMSG_FIRSTHDR(&m);
  c->cmsg_level = IPPROTO_IP;
  c->cmsg_type  = IP_PKTINFO;
  c->cmsg_len   = CMSG_LEN(sizeof(struct in_pktinfo));
  pi = (struct in_pktinfo *)CMSG_DATA(c);
  memset(pi, 0, sizeof(*pi));
  pi->ipi_ifindex  = r->ifindex;
  pi->ipi_spec_dst = r->ip;      /* source address for our replies/streams */
}

/* sendto pinned to radio r's interface + source IP (via its prebuilt IP_PKTINFO). */
static ssize_t send_from_radio(int sock, const void *buf, size_t len, radio_t *r,
                               const struct sockaddr_in *to)
{
  struct iovec iov = { (void *)buf, len };
  struct msghdr m;
  memset(&m, 0, sizeof(m));
  m.msg_name       = (void *)to;
  m.msg_namelen    = sizeof(*to);
  m.msg_iov        = &iov;
  m.msg_iovlen     = 1;
  m.msg_control    = r->cmsg.buf;
  m.msg_controllen = CMSG_SPACE(sizeof(struct in_pktinfo));
  return sendmsg(sock, &m, 0);
}

/* recvmsg that also reports which interface the datagram arrived on (ifindex). */
static ssize_t recv_ifindex(int sock, void *buf, size_t len, struct sockaddr_in *from,
                            unsigned *ifindex)
{
  cmsgbuf_t cbuf;
  struct iovec iov = { buf, len };
  struct msghdr m;
  struct cmsghdr *c;
  ssize_t n;
  memset(&m, 0, sizeof(m));
  m.msg_name       = from;
  m.msg_namelen    = sizeof(*from);
  m.msg_iov        = &iov;
  m.msg_iovlen     = 1;
  m.msg_control    = cbuf.buf;
  m.msg_controllen = sizeof(cbuf.buf);
  n = recvmsg(sock, &m, 0);
  *ifindex = 0;
  for(c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c))
    if(c->cmsg_level == IPPROTO_IP && c->cmsg_type == IP_PKTINFO)
      *ifindex = ((struct in_pktinfo *)CMSG_DATA(c))->ipi_ifindex;
  return n;
}

/* Build one P2 data packet for radio r's local DDC ch from a raw instant frame.
   The physical DDC column in the 128-byte instant is (r->ddc_base + ch)*8:
   I at bytes 0..3, Q at bytes 4..7 (26-bit signed in a 32-bit LE word). Emit Q then I,
   each 24-bit big-endian - exactly the byte order the old subset_1 hardware remap made. */
static inline void build_packet(uint8_t *pkt, const uint8_t *raw, radio_t *r, int ch)
{
  int i;
  uint32_t s = r->seq_ddc[ch]++;
  pkt[0] = s >> 24; pkt[1] = s >> 16; pkt[2] = s >> 8; pkt[3] = s;
  uint64_t ts = r->ts_ddc[ch]; r->ts_ddc[ch] += SAMPLES_PER_FRAME;
  for(i = 0; i < 8; ++i) pkt[4 + i] = (uint8_t)(ts >> (56 - 8 * i));
  pkt[12] = 0; pkt[13] = 24;
  pkt[14] = SAMPLES_PER_FRAME >> 8; pkt[15] = SAMPLES_PER_FRAME & 0xff;
  const uint8_t *sp = raw + (r->ddc_base + ch) * 8;
  uint8_t *dp = pkt + 16;
  for(i = 0; i < SAMPLES_PER_FRAME; ++i)
  {
    dp[0] = sp[6]; dp[1] = sp[5]; dp[2] = sp[4];   /* Q: bits [23:16],[15:8],[7:0] */
    dp[3] = sp[2]; dp[4] = sp[1]; dp[5] = sp[0];   /* I: bits [23:16],[15:8],[7:0] */
    dp += 6; sp += FIFO_WORD;
  }
}

/* Shared reader: copy instants from the coherent DDR ring into the SPSC frame ring while
   any radio is streaming. One consumer cursor follows the FPGA writer; it is resynced to
   the current writer position each time streaming (re)starts. */
void *reader_thread(void *arg)
{
  const uint32_t nbytes = SAMPLES_PER_FRAME * FIFO_WORD;   /* one packet: 238 * 128 */
  int active_prev = 0;
  (void)arg;
  while(1)
  {
    if(!any_active()) { active_prev = 0; usleep(1000); continue; }

    if(!active_prev)
    {
      uint32_t prod = *rx_wptr * DMA_BURST;    /* start at the current writer position */
      consumer = (prod / FIFO_WORD) * FIFO_WORD;
      __atomic_store_n(&ring_tail, 0, __ATOMIC_RELEASE);
      ring_head = 0;
      active_prev = 1;
    }

    uint32_t prod  = *rx_wptr * DMA_BURST;
    uint32_t avail = (prod + DMA_RING_BYTES - consumer) % DMA_RING_BYTES;

    if(avail > DMA_RING_BYTES - DMA_RING_BYTES / 4)
      { consumer = (prod / FIFO_WORD) * FIFO_WORD; continue; }   /* overflow: resync */

    if(avail < nbytes) { usleep(100); continue; }

    if(ring_head - __atomic_load_n(&ring_tail, __ATOMIC_ACQUIRE) >= RING_LEN)
      { usleep(50); continue; }                                 /* sender backpressure */

    raw_slot *fr = &ring[ring_head % RING_LEN];
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
    __atomic_store_n(&ring_head, ring_head + 1, __ATOMIC_RELEASE);
  }
  return NULL;
}

/* Shared sender: for each active radio, demux its enabled DDCs and sendmmsg to its client,
   each packet pinned (IP_PKTINFO) to that radio's interface + source IP. */
void *sender_thread(void *arg)
{
  static uint8_t txpkt[DDC_PER_RADIO][SEND_BATCH][PKT_SIZE];
  struct mmsghdr msgs[SEND_BATCH];
  struct iovec   iov[SEND_BATCH];
  uint32_t j, n;
  int ri, ch;
  (void)arg;

  while(1)
  {
    if(!any_active()) { usleep(1000); continue; }

    uint32_t head  = __atomic_load_n(&ring_head, __ATOMIC_ACQUIRE);
    uint32_t avail = head - ring_tail;
    if(avail == 0) { usleep(100); continue; }
    n = avail < SEND_BATCH ? avail : SEND_BATCH;

    for(ri = 0; ri < NUM_RADIOS; ++ri)
    {
      radio_t *r = &radios[ri];
      if(!(r->running && r->have_host)) continue;
      uint32_t enable = r->ddc_enable;
      for(ch = 0; ch < DDC_PER_RADIO; ++ch)
      {
        if(!(enable & (1u << ch))) continue;
        for(j = 0; j < n; ++j)
        {
          raw_slot *fr = &ring[(ring_tail + j) % RING_LEN];
          build_packet(txpkt[ch][j], fr->raw, r, ch);
          iov[j].iov_base = txpkt[ch][j];
          iov[j].iov_len  = PKT_SIZE;
          memset(&msgs[j], 0, sizeof(msgs[j]));
          msgs[j].msg_hdr.msg_name       = &r->host_addr;
          msgs[j].msg_hdr.msg_namelen    = sizeof(r->host_addr);
          msgs[j].msg_hdr.msg_iov        = &iov[j];
          msgs[j].msg_hdr.msg_iovlen     = 1;
          msgs[j].msg_hdr.msg_control    = r->cmsg.buf;
          msgs[j].msg_hdr.msg_controllen = CMSG_SPACE(sizeof(struct in_pktinfo));
        }
        sendmmsg(sock_data[ch], msgs, n, 0);
      }
    }
    __atomic_store_n(&ring_tail, ring_tail + n, __ATOMIC_RELEASE);
  }
  return NULL;
}

/* ---------- high-priority status to the host (port 1025), ~50 ms, per radio ---------- */
void *status_thread(void *arg)
{
  radio_t *r = (radio_t *)arg;
  uint8_t st[60];
  while(1)
  {
    usleep(50000);
    if(!r->have_host) continue;

    memset(st, 0, sizeof(st));
    uint32_t s = r->seq_status++;
    st[0] = s >> 24; st[1] = s >> 16; st[2] = s >> 8; st[3] = s;
    st[4] = 0x10;                 /* bit4: PLL locked (we have no external 10MHz) */
    send_from_radio(sock_ddcspec, st, sizeof(st), r, &r->host_addr);
  }
  return NULL;
}

//
// The microphone thread just sends silence, that is a "zeroed" mic frame every
// 1.333 msec and needs to be sent for some app's timing purposes. One per radio.
//
void *mic_thread(void *arg)
{
  radio_t *r = (radio_t *)arg;
  unsigned long seqnum = 0;
  unsigned char mic_buffer[132];
  struct timespec delay;

  memset(mic_buffer, 0, 132);
  clock_gettime(CLOCK_MONOTONIC, &delay);

  while(1)
  {
    if(!r->running || !r->have_host)
    {
      usleep(1000);
      seqnum = 0;
      clock_gettime(CLOCK_MONOTONIC, &delay);
      continue;
    }

    *(uint32_t *)mic_buffer = htonl(seqnum++);

    delay.tv_nsec += 1333333;    /* 64 samples at 48 kHz */
    while(delay.tv_nsec >= 1000000000)
    {
      delay.tv_nsec -= 1000000000;
      delay.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &delay, NULL);

    if(send_from_radio(sock_mic, mic_buffer, 132, r, &r->host_addr) < 0)
    {
      perror("***** ERROR: Mic thread sendto");
      break;
    }
  }
  return NULL;
}

/* ---------- discovery reply (from port 1024), per radio ---------- */
static void send_discovery_reply(radio_t *r, struct sockaddr_in *to)
{
  uint8_t reply[60];
  memset(reply, 0, sizeof(reply));
  /* bytes 0-3 sequence = 0 */
  reply[4]  = r->running ? 3 : 2;   /* 2 = available, 3 = in use */
  memcpy(reply + 5, r->mac, 6);     /* bytes 5-10 this interface's MAC */
  reply[11] = BOARD_TYPE;           /* Angelia */
  reply[12] = PROTOCOL_VERSION;
  reply[13] = CODE_VERSION;
  reply[20] = DDC_PER_RADIO;        /* # DDCs this radio presents -> client's rx count */
  reply[21] = 1;                    /* frequency sent as phase word */
  reply[22] = 0;                    /* big-endian, 3-byte I/Q */
  send_from_radio(sock_cmd, reply, sizeof(reply), r, to);
}

/* ---------- DDC-specific (port 1025): DDC-enable mask + per-DDC ADC assignment.
   Local DDC ch maps to physical DDC r->ddc_base + ch. The ADC-select bits for this
   radio's DDCs are written into the shared rx_sel register with a read-modify-write so
   the other radio's bits are untouched. Sample-rate fields are ignored (48-ksps only). */
static void process_ddc_specific(radio_t *r, const uint8_t *b)
{
  int ch, j, adc;
  uint32_t en = 0, sel_local = 0;
  uint32_t rmask = (DDC_PER_RADIO >= 32) ? 0xffffffffu : ((1u << DDC_PER_RADIO) - 1);

  for(j = 0; j < EN_BYTES; ++j) en |= (uint32_t)b[7 + j] << (8 * j);
  en &= rmask;
  r->ddc_enable = en;

  for(ch = 0; ch < DDC_PER_RADIO; ++ch)
  {
    if(!(en & (1u << ch))) continue;
    adc = b[17 + ch * 6];                             /* 0 = ADC0, 1 = ADC1 */
    sel_local |= (uint32_t)(adc & 1) << ch;
  }

  /* Apply the host's ADC bits only to DDCs left host-controlled; DDCs pinned via start.sh
     (adc_force_mask) keep their fixed ADC. */
  {
    uint32_t slice  = rmask << r->ddc_base;                  /* this radio's physical DDC bits */
    uint32_t host   = (sel_local << r->ddc_base) & ~adc_force_mask;   /* host bits, non-pinned only */
    uint32_t forced = adc_force_val & adc_force_mask & slice;         /* pinned bits for this radio */
    *rx_sel = (*rx_sel & ~slice) | forced | host;
  }
}

/* ---------- high-priority (port 1027): run bit + per-DDC phase words, per radio ---------- */
static void process_high_priority(radio_t *r, const uint8_t *b, ssize_t n)
{
  int ch;
  int newrun = b[4] & 1;
  for(ch = 0; ch < DDC_PER_RADIO; ++ch)
  {
    int phys = r->ddc_base + ch;
    uint32_t pinc = phaseword_to_pinc(be32(b + 9 + ch * 4));
    rx_freq[phys] = pinc ? pinc : park_pinc(phys);   /* never leave an NCO at 0 (see park_pinc) */
  }

  /* Open-collector / BCD band-filter outputs are a single board-global register, so only
     the primary radio (radio 0) drives them (P2 High-Priority byte 1401 -> E1 exp_p pins,
     ((x & 0x1e) << 3), matching the Protocol-1 transceiver). */
  if(r->index == 0 && n >= 1402) *rx_gpio = (uint8_t)((b[1401] & 0x1e) << 3);

  if(newrun && !r->running)   /* this radio's run-start edge: reset only its stream state */
  {
    for(ch = 0; ch < DDC_PER_RADIO; ++ch) { r->seq_ddc[ch] = 0; r->ts_ddc[ch] = 0; }
    /* the DMA writer runs continuously and is shared; the reader resyncs the ring cursor
       to the live writer position when streaming (re)starts (see reader_thread). */
  }
  r->running = newrun;
  clock_gettime(CLOCK_MONOTONIC, &r->last_cc);
}

/* Hold the DMA writer in reset before we exit, so it stops touching the CMA ring BEFORE
   the kernel frees it. Covers SIGTERM/SIGINT, so swap the server with `pkill -TERM`. */
static void on_signal(int sig)
{
  (void)sig;
  if(rx_rst) *rx_rst &= ~1;
  _exit(0);
}

static void get_if_mac(const char *ifname, uint8_t out[6])
{
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  struct ifreq r;
  memset(&r, 0, sizeof(r));
  strncpy(r.ifr_name, ifname, IFNAMSIZ - 1);
  if(ioctl(s, SIOCGIFHWADDR, &r) == 0) memcpy(out, r.ifr_addr.sa_data, 6);
  else fprintf(stderr, "warning: SIOCGIFHWADDR %s: %s\n", ifname, strerror(errno));
  close(s);
}

static struct in_addr get_if_ip(const char *ifname)
{
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  struct ifreq r;
  struct in_addr a;
  a.s_addr = 0;
  memset(&r, 0, sizeof(r));
  strncpy(r.ifr_name, ifname, IFNAMSIZ - 1);
  if(ioctl(s, SIOCGIFADDR, &r) == 0)
    a = ((struct sockaddr_in *)&r.ifr_addr)->sin_addr;
  close(s);
  return a;
}

/* A shared UDP socket bound to INADDR_ANY:port with IP_PKTINFO, so we can tell which
   interface each datagram arrived on and pin each reply to the right interface + source. */
static int mk_sock(int port)
{
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  int yes = 1;
  struct sockaddr_in a;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  setsockopt(s, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  setsockopt(s, IPPROTO_IP, IP_PKTINFO, &yes, sizeof(yes));
  memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons(port);
  if(bind(s, (struct sockaddr *)&a, sizeof(a)) < 0)
    fprintf(stderr, "warning: bind :%d: %s\n", port, strerror(errno));
  return s;
}

int main(int argc, char *argv[])
{
  int fd, i, ri, ch;
  volatile void *cfg, *sts;

  /* Optional per-DDC ADC assignment: NUM_DDC args (physical DDC0..15), each:
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
  close(fd);

  rx_rst  = (uint8_t  *)(cfg + 0);
  rx_freq = (uint32_t *)(cfg + 4);
  rx_sel  = (uint32_t *)(cfg + OFF_SEL);
  rx_min  = (uint32_t *)(cfg + OFF_MIN);
  rx_ring = (uint32_t *)(cfg + OFF_RING);
  rx_gpio = (uint8_t  *)(cfg + OFF_GPIO);
  rx_wptr = (uint32_t *)(sts + 0);

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

  /* sensible defaults: every NCO parked at a distinct nonzero out-of-passband tone so an
     unused/untuned DDC never sits at pinc 0 (see park_pinc) */
  *rx_sel  = adc_force_val & adc_force_mask;   /* pinned DDCs -> their ADC; rest ADC0 until host sets */
  *rx_gpio = 0;
  for(i = 0; i < NUM_DDC; ++i)
    rx_freq[i] = park_pinc(i);

  /* start the DMA writer streaming into the ring (runs continuously for all radios) */
  *rx_rst &= ~1; *rx_rst |= 1;

  /* shared sockets, one per UDP port (multi-homed via IP_PKTINFO) */
  sock_cmd      = mk_sock(PORT_COMMAND);
  sock_ddcspec  = mk_sock(PORT_DDC_SPECIFIC);
  sock_highprio = mk_sock(PORT_HIGH_PRIORITY);
  sock_mic      = mk_sock(PORT_MIC);
  for(ch = 0; ch < DDC_PER_RADIO; ++ch)
    sock_data[ch] = mk_sock(PORT_DDC_DATA0 + ch);

  /* Silence ICMP port-unreachable on the P2 host->radio ports these RX-only virtual radios
     do not service: 1028 (speaker/LR audio) and 1029.. (DUC / TX I&Q). Thetis streams these
     continuously even in pure RX. If the ports are unbound the board's kernel answers each
     datagram with ICMP port-unreachable; on Windows that latches WSAECONNRESET onto Thetis's
     shared RX socket and freezes its receive loop (sooner at higher sample rates; piHPSDR on
     Linux is immune). Bind throwaway sockets (never read) so the datagrams land silently and
     no ICMP is emitted. One INADDR_ANY bind per port covers both virtual radios' IPs. */
  for(i = PORT_HIGH_PRIORITY + 1; i < PORT_DDC_DATA0; ++i)   /* 1028..1034 */
    (void)mk_sock(i);

  /* set up each virtual radio: interface index/MAC/IP, DDC window, prebuilt IP_PKTINFO.
     Wait briefly for each interface's IPv4 (mvl0 is configured by dhcpcd after start.sh
     creates the macvlan), so replies carry a valid source address. */
  for(ri = 0; ri < NUM_RADIOS; ++ri)
  {
    radio_t *r = &radios[ri];
    int tries;
    r->index    = ri;
    r->ifname   = ifnames[ri];
    r->ddc_base = ri * DDC_PER_RADIO;
    r->ifindex  = if_nametoindex(r->ifname);
    get_if_mac(r->ifname, r->mac);
    for(tries = 0; tries < 100; ++tries)   /* up to ~10 s for the interface IP */
    {
      r->ip = get_if_ip(r->ifname);
      if(r->ip.s_addr != 0) break;
      usleep(100000);
    }
    if(!r->ifindex)
      fprintf(stderr, "warning: interface %s not found (radio %d will be unreachable)\n",
              r->ifname, ri);
    build_pktinfo(r);
    fprintf(stderr, "radio %d on %s (ifindex %u): MAC %02x:%02x:%02x:%02x:%02x:%02x, IP %s, DDC %d..%d\n",
            ri, r->ifname, r->ifindex,
            r->mac[0], r->mac[1], r->mac[2], r->mac[3], r->mac[4], r->mac[5],
            inet_ntoa(r->ip), r->ddc_base, r->ddc_base + DDC_PER_RADIO - 1);
  }

  /* threads: one shared reader + sender for the DMA ring; a status + mic thread per radio.
     Reader on core 0, everything else on core 1 (data read isolated; the rest is light). */
  {
    pthread_t rtid, stid, tid;
    cpu_set_t cs;
    pthread_create(&rtid, NULL, reader_thread, NULL);
    pthread_create(&stid, NULL, sender_thread, NULL);
    CPU_ZERO(&cs); CPU_SET(0, &cs); pthread_setaffinity_np(rtid, sizeof(cs), &cs);
    CPU_ZERO(&cs); CPU_SET(1, &cs); pthread_setaffinity_np(stid, sizeof(cs), &cs);
    pthread_detach(rtid); pthread_detach(stid);
    for(ri = 0; ri < NUM_RADIOS; ++ri)
    {
      pthread_create(&tid, NULL, status_thread, &radios[ri]);
      pthread_setaffinity_np(tid, sizeof(cs), &cs); pthread_detach(tid);
      pthread_create(&tid, NULL, mic_thread, &radios[ri]);
      pthread_setaffinity_np(tid, sizeof(cs), &cs); pthread_detach(tid);
    }
    sched_setaffinity(0, sizeof(cs), &cs);      /* main (command) thread -> core 1 */
  }

  while(1)
  {
    fd_set fds;
    struct timeval tv = { 0, 200000 };   /* 200 ms so the watchdog runs */
    int maxfd = sock_cmd;
    FD_ZERO(&fds);
    FD_SET(sock_cmd, &fds);
    FD_SET(sock_ddcspec, &fds);   if(sock_ddcspec  > maxfd) maxfd = sock_ddcspec;
    FD_SET(sock_highprio, &fds);  if(sock_highprio > maxfd) maxfd = sock_highprio;

    if(select(maxfd + 1, &fds, NULL, NULL, &tv) > 0)
    {
      uint8_t buffer[2048];
      struct sockaddr_in from;
      unsigned ifidx;
      radio_t *r;
      ssize_t n;

      if(FD_ISSET(sock_cmd, &fds))
      {
        n = recv_ifindex(sock_cmd, buffer, sizeof(buffer), &from, &ifidx);
        r = radio_by_ifindex(ifidx);
        if(n >= 5 && r && !(buffer[0] == 0xEF && buffer[1] == 0xFE))
        {
          /* SparkSDR/piHPSDR also broadcast a Protocol-1 (Metis) discovery (starts with the
             0xEFFE magic) alongside the Protocol-2 one. Its byte[4] is 0x00, which would
             otherwise be taken as a P2 "general" packet and re-point this radio's stream to
             the discovery socket's port, freezing an in-progress DDC. Skip any P1 packet
             (the guard above) on this P2-only receiver. */
          if(buffer[4] == 0x02)          /* discovery: reply as the radio on this interface */
            send_discovery_reply(r, &from);
          else if(buffer[4] == 0x00)     /* general: this radio's C&C source */
          {
            r->host_addr = from; r->have_host = 1;
            clock_gettime(CLOCK_MONOTONIC, &r->last_cc);
          }
        }
      }
      if(FD_ISSET(sock_ddcspec, &fds))
      {
        n = recv_ifindex(sock_ddcspec, buffer, sizeof(buffer), &from, &ifidx);
        r = radio_by_ifindex(ifidx);
        if(n >= 23 && r)
        {
          r->host_addr = from; r->have_host = 1;
          clock_gettime(CLOCK_MONOTONIC, &r->last_cc);
          process_ddc_specific(r, buffer);
        }
      }
      if(FD_ISSET(sock_highprio, &fds))
      {
        n = recv_ifindex(sock_highprio, buffer, sizeof(buffer), &from, &ifidx);
        r = radio_by_ifindex(ifidx);
        if(n >= 13 && r)
        {
          r->host_addr = from; r->have_host = 1;
          process_high_priority(r, buffer, n);
        }
      }
    }

    /* per-radio session watchdog: if no C&C for >1 s, drop RUN and forget the host, so we
       stop streaming data + status to a stale address. */
    {
      struct timespec t;
      clock_gettime(CLOCK_MONOTONIC, &t);
      for(ri = 0; ri < NUM_RADIOS; ++ri)
      {
        radio_t *r = &radios[ri];
        if(r->have_host &&
           (t.tv_sec - r->last_cc.tv_sec) + (t.tv_nsec - r->last_cc.tv_nsec) * 1e-9 > 1.0)
        {
          r->running = 0; r->have_host = 0;
        }
      }
    }
  }

  return EXIT_SUCCESS;
}
