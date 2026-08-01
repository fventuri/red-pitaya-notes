# Narrowband SDR receiver compatible with HPSDR (Protocol 2)

## Introduction

This is the narrowband, high-channel-count variant of the [openHPSDR Protocol 2 receiver](/sdr-receiver-hpsdr2/) for the [TRX-duo](https://www.wimo.com/en/trx-duo). It provides **sixteen** receivers at a fixed 48 kSPS, which is ideal for running many parallel narrow-band decoders (FT8, WSPR, CW) across the HF bands at the same time.

The openHPSDR Ethernet Protocol 2 is described in the documents in the [OpenHPSDR-Firmware/Protocol 2](https://github.com/TAPR/OpenHPSDR-Firmware/tree/master/Protocol%202) repository.

## Two virtual radios

Stock openHPSDR programs cap the number of receivers per radio (linhpsdr allows 8, Thetis 12). To make all sixteen DDCs usable without patching any client, the server presents them as **two independent openHPSDR radios of eight receivers each**, one per network interface:

- radio 0 on `eth0` (the board's normal IP address);
- radio 1 on a `macvlan` interface (`mvl0`) with its own MAC and IP address (obtained by DHCP).

A stock linhpsdr / piHPSDR / Thetis therefore discovers **two** separate 8-receiver radios and can connect to both simultaneously, for a total of sixteen receivers.

![Two virtual radios](/img/sdr-receiver-hpsdr2-narrow-radios.png)

> **Use a wired LAN connection.** Sixteen simultaneous I/Q streams are steady but relentless; a Wi-Fi link can drop packets under that load and cause audible distortion. Over a wired connection the board delivers all sixteen receivers cleanly.

## Hardware

The FPGA configuration consists of sixteen identical digital down-converters (DDC). The structure of one DDC is shown in the following diagram:

![Narrowband HPSDR2 receiver](/img/sdr-receiver-hpsdr2-narrow-ddc.png)

The TRX-duo has two ADC inputs, and the ADC (ADC0 or ADC1) feeding each DDC is individually selectable. The NCO, complex mixer and CIC are custom cores that are time-shared across all sixteen DDCs.

To fit sixteen DDCs on the Zynq 7010, each DDC uses a **0-DSP phase-truncation NCO** (`dds_phase`) instead of the interpolated DDS, freeing the DSP slices that would otherwise cap the receiver count. As of **v0.6.5** the NCO uses a 12-bit quarter-wave lookup table (14 effective phase bits), giving a worst-case spurious-free dynamic range of about **-84 dBc**.

The I/Q data rate is fixed at **48 kSPS**. The tunable frequency range covers from 0 Hz to 62.5 MHz.

The receiver uses a cache-coherent ACP/DMA data path to move the I/Q streams from the FPGA to the server. Because reprogramming the FPGA while the ACP port is active hangs the DMA, **reboot the board before switching to another openHPSDR Protocol 2 receiver (or to another application).**

The open-collector / BCD band-filter outputs (on the E1 extension connector) are driven from the Protocol 2 High-Priority "Open Collector Outputs" byte.

The [projects/sdr_receiver_hpsdr2_narrow_trx_duo](https://github.com/fventuri/red-pitaya-notes/tree/hpsdr2/projects/sdr_receiver_hpsdr2_narrow_trx_duo) directory contains the Tcl files ([block_design.tcl](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_narrow_trx_duo/block_design.tcl), [rx.tcl](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_narrow_trx_duo/rx.tcl)) that instantiate, configure and interconnect all the needed IP cores.

The [server](https://github.com/fventuri/red-pitaya-notes/tree/hpsdr2/projects/sdr_receiver_hpsdr2_narrow_trx_duo/server) directory contains the source code of the UDP server ([sdr-receiver-hpsdr2.c](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_narrow_trx_duo/server/sdr-receiver-hpsdr2.c)) that presents the two virtual radios and transmits the I/Q data streams to the SDR programs.

## Software

This receiver is known to work with the following programs that support the openHPSDR Ethernet Protocol 2:

- [linhpsdr](https://github.com/g0orx/linhpsdr) (Linux)
- [piHPSDR](https://github.com/g0orx/pihpsdr) (Linux / Raspberry Pi)
- [Thetis](https://github.com/ramdor/Thetis) (Windows)
- [SparkSDR](https://www.sparksdr.com) (Windows / Linux / macOS), which can run several decoders across the receivers

[SDR-Console](https://www.sdr-radio.com) also connects, but only uses a single receiver.

## Getting started

- Connect an antenna to the ADC0 (IN1) connector on the TRX-duo board.
- Download the SD card image from the [v0.6.5 release](https://github.com/fventuri/red-pitaya-notes/releases/tag/v0.6.5). The image is split into several files (`.z01`–`.z04` plus the final `.zip`); download all of them and unzip the set together. More details about the SD card image can be found at [this link](/alpine/).
- Copy the contents of the SD card image zip file to a micro SD card.
- Optionally, to start the application automatically at boot time, copy its `start.sh` file from `apps/sdr_receiver_hpsdr2_narrow_trx_duo` to the topmost directory on the SD card.
- Install the micro SD card in the TRX-duo board, connect it to your network with a wired LAN cable, and connect the power.
- Install and run one of the openHPSDR Protocol 2 programs listed above. It will discover two 8-receiver radios; connect to both to use all sixteen receivers.

## Building from source

The installation of the development machine is described at [this link](/development-machine/).

The structure of the source code and of the development chain is described at [this link](/led-blinker/).

Setting up the Vitis and Vivado environment:

```bash
source /opt/Xilinx/2026.1/Vitis/settings64.sh
```

Cloning the source code repository (the openHPSDR Protocol 2 receivers live on the `hpsdr2` branch):

```bash
git clone -b hpsdr2 https://github.com/fventuri/red-pitaya-notes
cd red-pitaya-notes
```

Building `sdr_receiver_hpsdr2_narrow_trx_duo.bit`:

```bash
make NAME=sdr_receiver_hpsdr2_narrow_trx_duo bit
```

Building SD card image zip file:

```bash
source helpers/build-all.sh
```
