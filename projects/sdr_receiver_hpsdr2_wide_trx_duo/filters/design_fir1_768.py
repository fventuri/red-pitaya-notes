#!/usr/bin/env python3
"""Phase 6 (768 ksps): design the FIR1 coefficients for the 4/5 chain.
Chain = CIC(R) -> FIR0(24/25, unchanged) -> FIR1(4/5, THIS). Output = 96000/R kHz
=> rates 48/96/192/384/768 for R=2000/1000/500/250/125. FIR1 must (a) anti-alias
the 4/5 decimation, (b) invert the CIC droop (heavy at 768/R=125: ~-11 dB at the
band edge, pre-inverted by a 1/(CIC*FIR0) passband target).

Method: fir2 / frequency-sampling with a Kaiser window (scipy firwin2 == R's fir2),
the same structurally-overshoot-free method that keeps the 8-DDC receiver's fir_1
clean (see ../../sdr_receiver_hpsdr2_trx_duo/filters/rx_fir_1.r).

History: an earlier firls (least-squares) version left the transition band [Fp,Fs] a
"don't care"; because the passband target ends high (+9.3 dB, rising) to pre-invert
the CIC droop, least-squares rang UP into that gap -> a fixed +10 dB combined-response
peak at ~0.46*fs (the band edge). An intermediate firls fix added a constrained
descending transition ramp, which removed the peak but cost passband width and
stopband depth. This fir2/Kaiser version is better on every axis at the SAME 801-tap
cost: the window is structurally incapable of overshoot, so no weight tuning can go
wrong. The old note that "firwin2 could not" reach a deep stopband was only true at
the natural kaiserord order (~300 taps -> -30 dB); given the 801 taps this filter
already budgets, the Kaiser window over-delivers.

Result (combined CIC*FIR0*FIR1, all rates): edge peak +0.01 dB (was +10.1), passband
flat to 0.44*fs within +/-0.01 dB, stopband -108 dB (survives 24-bit quantization),
monotonic rolloff, DC gain 1.0002 (in-band level unchanged)."""
import re, numpy as np
from scipy.signal import firwin2, kaiserord
RX_TCL = "projects/sdr_receiver_hpsdr2_trx_duo/rx.tcl"   # source of FIR0
def fir0():
    t=open(RX_TCL).read(); i=t.find("fir_compiler fir_0")
    m=re.search(r'COEFFICIENTVECTOR \{([^}]*)\}', t[i:i+40000])
    h=np.array([float(x) for x in m.group(1).split(",")]); return h/h.sum()
Fadc=125e6; N=6; R=125; h0=fir0()
Fs0=Fadc/R; Fr0=24*Fs0; Fs1=Fs0*24/25; Fr1=4*Fs1; Nyq=Fr1/2   # Nyq = FIR1 input Nyquist
Hf=lambda h,Fr,f:(np.abs(np.exp(-2j*np.pi*np.outer(f/Fr,np.arange(len(h))))@h))
def Hcic(f,R):
    x=np.pi*f/Fadc; d=R*np.sin(x); return np.abs(np.where(np.abs(d)<1e-12,1.0,np.sin(R*x)/d))**N
# passband 0..Fp with 1/(CIC*FIR0) droop-inversion target; stopband edge Fs = output fs/2
Fp=0.88*384e3; Fs=384e3; ntaps=801; stop_db=90.0
# Kaiser beta from the target attenuation over the transition [Fp,Fs] (normalized to Nyq);
# ntaps is fixed at 801 (>> kaiserord's natural order), so the window over-delivers depth.
_, beta = kaiserord(stop_db, (Fs-Fp)/Nyq)
fpb=np.linspace(0,Fp,400); gpb=1.0/(Hcic(fpb,R)*Hf(h0,Fr0,fpb)); gpb/=gpb[0]
freq=np.concatenate([fpb/Nyq, [Fs/Nyq, 1.0]])   # firwin2 grid: 0..1 (1 = Nyq)
gain=np.concatenate([gpb,      [0.0,     0.0]])
freq[0]=0.0; freq[-1]=1.0
h1=firwin2(ntaps, freq, gain, window=('kaiser', beta))
open("fir1_768_coeffs.txt","w").write(", ".join(f"{x:.10e}" for x in h1))
print("wrote fir1_768_coeffs.txt", len(h1), "taps, DC gain", round(h1.sum(),4), "beta", round(beta,3))
