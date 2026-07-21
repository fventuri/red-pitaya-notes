#!/usr/bin/env python3
"""EXTRAWIDE 4-DDC / 1536 kHz receiver — filter design + characterization (EXPLORATION).

This is the consolidated design script from the 2026-07-20 feasibility exploration
(see docs/HANDOFF_EXTRAWIDE_1536.md). The extrawide project is NOT built yet; this
script designs and characterizes the three FIR stages and prints the resource/alias
tradeoff so a build can be committed to a chosen operating point.

Chain (full-BW, exact 1536 kHz):
    ADC 125M -> CIC(R=50, N=6) -> FIR0 24/25 -> FIR1 4/5 -> FIR2 4/5 (sharp+droop) = 1536k
    rates:      2500k             2400k          1920k       1536k
  - R=50 (not 75): keeps CIC-Nyquist at 1250k so the passband droop at the band edge is
    only ~-6 dB (R=75 gives -14 dB, too steep to compensate flat). NOTE: R=50 is below the
    stock CIC MINIMUM_RATE 125 -> the extrawide CIC must be reconfigured for MIN_RATE 50.
  - Droop compensation lives in the sharp final FIR2 via fir2/Kaiser (the proven method
    from the wide-receiver fir_1 edge-peak fix). FIR0/FIR1 are plain clean lowpasses.
  - Usable BW is capped by CIC-Nyquist, NOT the final rate. Guard band FPASS..768k is
    unused, so FIR2's stopband edge is Fout-FPASS.

KEY RESULT (2026-07-20, from ACTUAL builds on xc7z010clg400-1 — see HANDOFF §0.5):
the binding knob is FPASS (it sets FIR2's stopband edge = Fout-FPASS, which sets the
band-edge alias), NOT the FIR0/FIR1 stopband depth ATTEN01. Raise FPASS to move the edge
and buy bandwidth; the edge alias is FIR2-floored, so ATTEN01 can stay low to save DSP.

  point   BW      FPASS   built?  DSP     WNS       verdict
  0.42fs  645k    700k    YES     60/80   +0.146ns  comfortable
  0.44fs  676k    720k    YES     68/80   +0.397ns  comfortable
  0.45fs  691k    730k    YES     74/80   +0.293ns  fits — the 7010 CEILING (this default)
  0.46fs  707k    740k    NO      83/80   (DRC)     over by 3 DSP -> 7020

The DSP model below runs ~20% HIGH vs real synthesis (predicted 74/85/107 for
0.44/0.45/0.46; actual 68/74/83). 0.45*fs = 691 kHz is the honest 7010 maximum:
92.5% DSP, timing clean, alias -129 dB. Passband flat to 0.40*fs regardless.
The old analysis-era claim "0.44*fs not reachable / needs 7020" was wrong (model pessimism
+ never raising FPASS). Wall at 0.46: FIR2 transition (Fout-2*FPASS) collapses -> 785 taps
-> 83 DSP, and guard band (28k) + CIC droop (-7.7dB) also max out together.

Halfband 4-stage was tried and does NOT help: exact-1536 can't feed a clean 3072k
halfband by decimation, forcing an interp-32 up-stage that eats the saving.

DSP model (calibrated to current wide FIR1: 801t/960k/12ch -> ~37 DSP):
    FIR_DSP ~= (taps/2) * input_rate * (2*N_DDC) / 125e6
"""
import numpy as np
from scipy.signal import firwin2, kaiserord

# ---- chain rates ----
Fadc=125e6; N=6; R=50
Fs0=Fadc/R                                    # 2500k  CIC out (Nyquist 1250k)
I0,D0=24,25; Fr0=I0*Fs0; Fs1=Fr0/D0           # FIR0 60M internal  -> 2400k
I1,D1=4,5;   Fr1=I1*Fs1; Fs2=Fr1/D1           # FIR1 9.6M          -> 1920k
I2,D2=4,5;   Fr2=I2*Fs2; Fout=Fr2/D2          # FIR2 7.68M         -> 1536k
N_DDC=4; Nch=2*N_DDC

# ---- operating point (default = the built 0.45*fs 7010 CEILING) ----
# FPASS is the binding knob: FIR2 stopband edge = Fout-FPASS. Built points:
#   700k->0.42fs/60DSP  720k->0.44fs/68DSP  730k->0.45fs/74DSP(ceiling)  740k->0.46fs/83DSP(FAILS)
FPASS   = 730e3   # usable BW edge; passband stays flat to 0.40fs. 730k = 0.45fs = 691k usable.
ATTEN01 = 44.0    # FIR0/FIR1 stopband dB: edge alias is FIR2-floored, so keep low to save DSP.
ATTEN2  = 90.0    # FIR2 (sharp) stopband dB

Hf=lambda h,Fr,f: np.abs(np.exp(-2j*np.pi*np.outer(np.atleast_1d(f)/Fr,np.arange(len(h))))@h)
def Hcic(f):
    with np.errstate(invalid='ignore',divide='ignore'):
        x=np.pi*np.atleast_1d(f)/Fadc; d=R*np.sin(x)
        return np.abs(np.where(np.abs(d)<1e-12,1.0,np.sin(R*x)/d))**N
def kaiser_lp(Fr,Fp,Fstop,stop_db):
    w=(Fstop-Fp)/(Fr/2); n,b=kaiserord(stop_db,w); n=int(n)|1
    return firwin2(n,[0,Fp/(Fr/2),Fstop/(Fr/2),1.0],[1,1,0,0],window=('kaiser',b))
def fir2_droop(Fr,Fp,Fstop,stop_db,Hpre):
    """sharp lowpass + 1/Hpre droop-comp passband, fir2/Kaiser (proven no-overshoot)."""
    w=(Fstop-Fp)/(Fr/2); n,beta=kaiserord(stop_db,w); n=int(n)|1
    fpb=np.linspace(0,Fp,400); g=1.0/Hpre(fpb); g/=g[0]
    freq=np.concatenate([fpb/(Fr/2),[Fstop/(Fr/2),1.0]]); gain=np.concatenate([g,[0,0]])
    freq[0]=0.0; freq[-1]=1.0
    return firwin2(n,freq,gain,window=('kaiser',beta))

# FIR0/FIR1: plain clean lowpass (fold-driven stopbands, lean on CIC rolloff for depth)
h0=kaiser_lp(Fr0, FPASS, 1050e3, ATTEN01)     # FIR0 antialias (fold at Fs1-FPASS)
h1=kaiser_lp(Fr1, FPASS, 1100e3, ATTEN01)     # FIR1 antialias
# FIR2: SHARP + droop comp for the whole chain (CIC*FIR0*FIR1); stopband edge = Fout-FPASS
Hpre=lambda f: Hcic(f)*Hf(h0,Fr0,f)*Hf(h1,Fr1,f)
h2=fir2_droop(Fr2, FPASS, Fout-FPASS, ATTEN2, Hpre)

def dsp(h,Fin): return (len(h)/2)*Fin*Nch/125e6
print(f"chain: CIC(R={R})->2500k  FIR0 24/25->2400k  FIR1 4/5->1920k  FIR2 4/5->{Fout/1e3:.0f}k")
print(f"CIC droop @ {FPASS/1e3:.0f}k = {20*np.log10(Hcic(np.array([FPASS]))[0]):+.1f} dB")
for nm,h,Fin in [("FIR0 24/25",h0,Fs0),("FIR1 4/5",h1,Fs1),("FIR2 4/5",h2,Fs2)]:
    print(f"  {nm:11s} taps={len(h):4d} DCgain={h.sum():.4f} estDSP={dsp(h,Fin):.1f}")
Dfir=dsp(h0,Fs0)+dsp(h1,Fs1)+dsp(h2,Fs2)
print(f"  FIR DSP(4DDC)~{Dfir:.0f}  + per-DDC(mix{Nch}+dds{N_DDC}+gain3~{Nch+N_DDC+3}) => TOTAL ~{Dfir+Nch+N_DDC+3:.0f} DSP  (7010=80, 7020=220)")

# characterization: passband + alias rejection vs claimed usable BW
f=np.linspace(0,20e6,800000)
H=Hcic(f)*Hf(h0,Fr0,f)*Hf(h1,Fr1,f)*Hf(h2,Fr2,f); H/=H[0]; db=20*np.log10(np.maximum(H,1e-12))
print(f"\npassband 0-{0.40*Fout/1e3:.0f}k (0.40fs): ripple={db[f<=0.40*Fout].max()-db[f<=0.40*Fout].min():.2f} dB  peak={db[f<=0.40*Fout].max():+.2f} dB")
fout=np.abs(((f+Fout/2)%Fout)-Fout/2)
print("alias rejection vs claimed usable BW (worst spur folding INTO 0..BW):")
for bwf in [0.30,0.34,0.36,0.38,0.40]:
    bw=bwf*Fout; prot=(f>Fout-bw+15e3)&(fout<=bw)
    print(f"  0-{bw/1e3:.0f}k ({bwf:.2f}fs): {db[prot].max():+.1f} dB")

# write coefficient files (paste into rx.tcl COEFFICIENTVECTOR when project is created)
for nm,h in [("fir0",h0),("fir1",h1),("fir2",h2)]:
    open(f"extrawide_{nm}_coeffs.txt","w").write(", ".join(f"{x:.10e}" for x in h))
print("\nwrote extrawide_fir0/fir1/fir2_coeffs.txt")
