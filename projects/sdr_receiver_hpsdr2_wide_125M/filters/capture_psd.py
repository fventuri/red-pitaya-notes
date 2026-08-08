#!/usr/bin/env python3
"""Capture receiver noise IQ over UDP and compute an averaged (Welch) PSD to
characterize the noise-floor SHAPE across the band. Usage:
  capture_psd.py <ip> <N> <rate_khz> <secs> <tag> [ddc_list_csv]
Writes <tag>.json = {tag,rate,fs,freq[],psd_db[],metrics{}} (PSD normalized so
the center-band median = 0 dB). Sample = I + jQ; per HPSDR packet the 6-byte
group is Q(24b BE) then I(24b BE)."""
import socket, struct, sys, time, json
import numpy as np

BOARD=sys.argv[1]; N=int(sys.argv[2]); RATE=int(sys.argv[3]); SECS=float(sys.argv[4]); TAG=sys.argv[5]
ANALYZE=[int(x) for x in sys.argv[6].split(",")] if len(sys.argv)>6 else list(range(N))
BASE,STEP=7_000_000,100_000; HPSDR_CLK=122_880_000
freqs=[BASE+ch*STEP for ch in range(N)]; adcs=[ch%2 for ch in range(N)]; enable=sum(1<<ch for ch in range(N))
fs=RATE*1000.0

s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
try: s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,64*1024*1024)
except OSError: pass
s.bind(("",0)); s.settimeout(2.0)
def snd(p,b): s.sendto(b,(BOARD,p))
d=bytearray(60); d[4]=2; snd(1024,d)
r,_=s.recvfrom(2048); print(f"[{TAG}] discovery num_ddc={r[20]}  rate={RATE}k fs={fs:.0f}")
g=bytearray(60); g[4]=0; g[37]=8; snd(1024,g)
ds=bytearray(1444); ds[4]=2; ds[7]=enable
for ch in range(N): ds[17+ch*6]=adcs[ch]; ds[18+ch*6]=(RATE>>8)&0xff; ds[19+ch*6]=RATE&0xff
ds[22]=24; snd(1025,ds)
hp=bytearray(1444); hp[4]=1
for ch in range(N):
    pw=(freqs[ch]<<32)//HPSDR_CLK; hp[9+ch*4:13+ch*4]=struct.pack(">I",pw&0xffffffff)
snd(1027,hp); time.sleep(0.3)
s.setblocking(False)
try:
    while True: s.recvfrom(4096)
except BlockingIOError: pass
s.setblocking(True); s.settimeout(2.0)

# collect per-DDC (seq -> 238 complex samples) for analyzed DDCs
buf={ch:{} for ch in ANALYZE}
t0=time.time(); nka=t0+0.05
while time.time()-t0<SECS:
    if time.time()>nka: snd(1027,hp); nka+=0.05
    try: p,frm=s.recvfrom(4096)
    except socket.timeout: continue
    sp=frm[1]
    if not(1035<=sp<=1042) or len(p)!=1444: continue
    ch=sp-1035
    if ch not in buf: continue
    seq=(p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3]
    raw=np.frombuffer(p,dtype=np.uint8,count=1444)
    body=raw[16:16+238*6].reshape(238,6).astype(np.int32)
    Q=(body[:,0]<<16)|(body[:,1]<<8)|body[:,2]
    I=(body[:,3]<<16)|(body[:,4]<<8)|body[:,5]
    Q=np.where(Q>=(1<<23),Q-(1<<24),Q); I=np.where(I>=(1<<23),I-(1<<24),I)
    buf[ch][seq]=(I+1j*Q).astype(np.complex64)
snd(1027,bytearray(1444))

# Welch PSD averaged over segments and over analyzed DDCs
SEG=8192; HOP=SEG//2; win=np.hanning(SEG); wpow=(win**2).sum()
acc=np.zeros(SEG); nseg=0
for ch in ANALYZE:
    seqs=sorted(buf[ch])
    if not seqs: continue
    # concatenate in seq order (rare gaps -> minor discontinuity, fine for noise PSD)
    x=np.concatenate([buf[ch][q] for q in seqs]).astype(np.complex128)
    i=0
    while i+SEG<=len(x):
        seg=x[i:i+SEG]*win
        P=np.abs(np.fft.fftshift(np.fft.fft(seg)))**2/(wpow*fs)
        acc+=P; nseg+=1; i+=HOP
psd=acc/max(nseg,1)
psd_db=10*np.log10(psd+1e-30)
freq=np.fft.fftshift(np.fft.fftfreq(SEG,d=1/fs))

# normalize to center-band median
cmask=np.abs(freq)<0.20*fs
center=np.median(psd_db[cmask]); psd_db-=center
def band(lo,hi):
    m=(freq>=lo*fs)&(freq<hi*fs); return float(np.median(psd_db[m]))
metrics=dict(nseg=nseg, samples_per_ddc={str(c):int(sum(len(buf[c][q]) for q in buf[c])) for c in ANALYZE},
    center=0.0, low_edge=band(-0.49,-0.45), high_edge=band(0.45,0.49),
    at_40pct_hi=band(0.38,0.42), at_45pct_hi=band(0.44,0.46), peak_edge=float(np.max(psd_db[np.abs(freq)>0.40*fs])))
json.dump(dict(tag=TAG,rate=RATE,fs=fs,N=N,analyze=ANALYZE,freq=freq.tolist(),psd_db=psd_db.tolist(),metrics=metrics),
          open(f"{TAG}.json","w"))
print(f"[{TAG}] segs={nseg}  center=0.0 dB (ref)")
print(f"[{TAG}] edge(low/high)= {metrics['low_edge']:+.1f} / {metrics['high_edge']:+.1f} dB   "
      f"@±40%={metrics['at_40pct_hi']:+.1f}  peak-in-outer40%= {metrics['peak_edge']:+.1f} dB")
