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

/* ---- configuration (parametrized; the wide sibling overrides NUM_DDC) ---- */
#define NUM_DDC            6       /* DDCs implemented in the FPGA           */
#define BOARD_TYPE         3       /* linhpsdr device enum: Angelia (2 ADCs) */
#define CODE_VERSION       1       /* firmware/code version (discovery [13]) */
#define PROTOCOL_VERSION   39      /* openHPSDR protocol version *10 ([12])  */
#define SAMPLES_PER_FRAME  238     /* 24-bit I/Q pairs per DDC packet        */
#define FIFO_WORD          48      /* bytes per instant in the DDR ring: 12 * 4B LE
                                      channels (DDC0_I, DDC0_Q, DDC1_I ... DDC5_Q),
                                      each a 26-bit signed value in a 32-bit LE word */
#define RATE_BASE          96000   /* rx_rate = RATE_BASE/rate_khz; 48000 for the
                                      2/5 FIR chain (48-384), 96000 for 4/5 (48-768) */

#define ADC_CLOCK          125000000.0
#define HPSDR_DSP_CLOCK    122880000.0    /* clients hardcode this           */

/* ---- Protocol-2 UDP ports ---- */
#define PORT_COMMAND       1024    /* discovery/general/command in; reply out */
#define PORT_DDC_SPECIFIC  1025    /* DDC-specific in; high-priority status out */
#define PORT_MIC           1026
#define PORT_HIGH_PRIORITY 1027    /* high-priority in (run + phase words)     */
#define PORT_DDC_DATA0     1035    /* DDC n I/Q out from source port 1035+n    */

/* ---- FPGA register windows ---- */
volatile uint8_t  *rx_rst;         /* cfg+0 bit0: writer/stream reset (active low) */
volatile uint8_t  *rx_sel;         /* cfg+1: per-DDC ADC select bitmap        */
volatile uint16_t *rx_rate;        /* cfg+2: shared CIC decimation word       */
volatile uint32_t *rx_freq;        /* cfg+4: rx_freq[NUM_DDC] phase increments */
volatile uint32_t *rx_min;         /* cfg+28: DDR ring physical base   (min_addr, cfg[255:224]) */
volatile uint32_t *rx_ring;        /* cfg+32: ring size-1 in 128B bursts       (cfg[287:256]) */
volatile uint8_t  *rx_gpio;        /* cfg+44: open-collector outputs -> E1 exp_p pins (cfg[359:352]) */
volatile uint32_t *rx_wptr;        /* sts+0: writer pointer, in 128-byte bursts */
volatile uint8_t  *dma_ram;        /* mmap of the CMA DDR ring (ACP-coherent)  */

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

/* ---------- data path: the FPGA DMAs 48-byte instants into a DDR ring over the
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
#define DMA_RING_INSTANTS (DMA_RING_BYTES / FIFO_WORD)     /* 65536 instants           */
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
   axis_gain between fir_1 and conv_1 — the samples arrive here already at the right
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
  /* Each raw instant is 12 * 4B LE channels; DDC ch has I at sp[0..3], Q at sp[4..7]
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

void *reader_thread(void *arg)
{
  int ch;
  (void)arg;
  const uint32_t nbytes = SAMPLES_PER_FRAME * FIFO_WORD;   /* one packet: 238 * 48 */
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
       the ring end (the ring is a whole multiple of the 48-byte instant). */
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
  struct mmsghdr msgs[SEND_BATCH];
  struct iovec   iov[SEND_BATCH];
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

    /* demux the sender's share (channels >= READER_DEMUX) */
    for(j = 0; j < n; ++j)
    {
      raw_slot *fr = &ring[(ring_tail + j) % RING_LEN];
      for(ch = READER_DEMUX; ch < NUM_DDC; ++ch)
        if(enable & (1u << ch)) build_packet(txpkt[ch][j], fr->raw, ch);
    }
    /* one sendmmsg per enabled DDC: reader-built packets live in the ring slot,
       sender-built ones in txpkt */
    for(ch = 0; ch < NUM_DDC; ++ch)
    {
      if(!(enable & (1u << ch))) continue;
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
static void process_ddc_specific(const uint8_t *b)
{
  int ch, adc, rate_khz;
  uint8_t sel = 0;
  uint16_t rate = *rx_rate;

  ddc_enable = b[7];             /* DDC-enable bitmap (DDC0..7) */

  for(ch = 0; ch < NUM_DDC; ++ch)
  {
    if(!(ddc_enable & (1u << ch))) continue;
    adc      = b[17 + ch * 6];                        /* 0 = ADC0, 1 = ADC1 */
    rate_khz = (b[18 + ch * 6] << 8) | b[19 + ch * 6];
    sel |= (adc & 1) << ch;
    if(rate_khz > 0) rate = (uint16_t)(RATE_BASE / rate_khz);  /* shared: last enabled wins */
  }

  *rx_sel  = sel;
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

  if((fd = open("/dev/mem", O_RDWR)) < 0) { perror("open /dev/mem"); return EXIT_FAILURE; }
  cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x40000000);
  sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x41000000);
  close(fd);

  rx_rst  = (uint8_t  *)(cfg + 0);
  rx_sel  = (uint8_t  *)(cfg + 1);
  rx_rate = (uint16_t *)(cfg + 2);
  rx_freq = (uint32_t *)(cfg + 4);
  rx_min  = (uint32_t *)(cfg + 28);
  rx_ring = (uint32_t *)(cfg + 32);
  rx_gpio = (uint8_t  *)(cfg + 44);
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
  *rx_sel  = 0;
  *rx_gpio = 0;                             /* open-collector / filter pins low */
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
    pthread_t rtid, stid, sttid, mtid;
    cpu_set_t cs;
    pthread_create(&rtid,  NULL, reader_thread, NULL);
    pthread_create(&stid,  NULL, sender_thread, NULL);
    pthread_create(&sttid, NULL, status_thread, NULL);
    pthread_create(&mtid,  NULL, mic_thread, NULL);
    CPU_ZERO(&cs); CPU_SET(0, &cs); pthread_setaffinity_np(rtid, sizeof(cs), &cs);
    CPU_ZERO(&cs); CPU_SET(1, &cs);
    pthread_setaffinity_np(stid,  sizeof(cs), &cs);
    pthread_setaffinity_np(sttid, sizeof(cs), &cs);
    pthread_setaffinity_np(mtid, sizeof(cs), &cs);
    sched_setaffinity(0, sizeof(cs), &cs);      /* main (command) thread -> core 1 */
    pthread_detach(rtid); pthread_detach(stid); pthread_detach(sttid); pthread_detach(mtid);
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
          }
        }
      }
      if(FD_ISSET(sock_ddcspec, &fds))
      {
        n = recvfrom(sock_ddcspec, buffer, sizeof(buffer), 0, (struct sockaddr *)&from, &fromlen);
        if(n >= 23) { host_addr = from; have_host = 1; clock_gettime(CLOCK_MONOTONIC, &last_cc); process_ddc_specific(buffer); }
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
