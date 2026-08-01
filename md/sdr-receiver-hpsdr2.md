# SDR receiver compatible with HPSDR (Protocol 2)

## Introduction

This version of the SDR receiver for the [TRX-duo](https://www.wimo.com/en/trx-duo) emulates an openHPSDR module with up to eight receivers using the newer **Ethernet Protocol 2**. It may be useful for programs that support the openHPSDR Protocol 2 communication protocol.

The openHPSDR Ethernet Protocol 2 is described in the documents in the [OpenHPSDR-Firmware/Protocol 2](https://github.com/TAPR/OpenHPSDR-Firmware/tree/master/Protocol%202) repository.

## Hardware

The FPGA configuration consists of up to eight identical digital down-converters (DDC). Their structure is shown in the following diagram:

![HPSDR2 receiver](/img/sdr-receiver-hpsdr2-ddc.png)

The TRX-duo has two ADC inputs, and the ADC (ADC0 or ADC1) feeding each DDC is individually selectable, so the eight receivers can be split between the two antenna inputs.

The main problem in emulating the openHPSDR hardware is that the TRX-duo ADC sample rate is 125 MSPS while the openHPSDR reference sample rate is 122.88 MSPS. To address this, the receiver contains a set of FIR filters for fractional sample rate conversion.

The resulting I/Q data rate is configurable and three settings are available: **48, 96, 192 kSPS**. The tunable frequency range covers from 0 Hz to 62.5 MHz.

The receiver uses a cache-coherent ACP/DMA data path to move the I/Q streams from the FPGA to the server. Because reprogramming the FPGA while the ACP port is active hangs the DMA, **reboot the board before switching to another openHPSDR Protocol 2 receiver (or to another application).**

The open-collector / BCD band-filter outputs (on the E1 extension connector) are driven from the Protocol 2 High-Priority "Open Collector Outputs" byte, so automatic band-pass / low-pass filter boards can be switched exactly as with Protocol 1.

The [projects/sdr_receiver_hpsdr2_trx_duo](https://github.com/fventuri/red-pitaya-notes/tree/hpsdr2/projects/sdr_receiver_hpsdr2_trx_duo) directory contains the Tcl files ([block_design.tcl](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_trx_duo/block_design.tcl), [rx.tcl](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_trx_duo/rx.tcl)) that instantiate, configure and interconnect all the needed IP cores.

The [server](https://github.com/fventuri/red-pitaya-notes/tree/hpsdr2/projects/sdr_receiver_hpsdr2_trx_duo/server) directory contains the source code of the UDP server ([sdr-receiver-hpsdr2.c](https://github.com/fventuri/red-pitaya-notes/blob/hpsdr2/projects/sdr_receiver_hpsdr2_trx_duo/server/sdr-receiver-hpsdr2.c)) that receives control commands and transmits the I/Q data streams to the SDR programs.

## Software

This receiver is known to work with the following programs that support the openHPSDR Ethernet Protocol 2:

- [linhpsdr](https://github.com/g0orx/linhpsdr) (Linux)
- [piHPSDR](https://github.com/g0orx/pihpsdr) (Linux / Raspberry Pi)
- [Thetis](https://github.com/ramdor/Thetis) (Windows)

## Getting started

- Connect an antenna to the ADC0 (IN1) connector on the TRX-duo board.
- Download the SD card image from the [v0.6.5 release](https://github.com/fventuri/red-pitaya-notes/releases/tag/v0.6.5). The image is split into several files (`.z01`–`.z04` plus the final `.zip`); download all of them and unzip the set together. More details about the SD card image can be found at [this link](/alpine/).
- Copy the contents of the SD card image zip file to a micro SD card.
- Optionally, to start the application automatically at boot time, copy its `start.sh` file from `apps/sdr_receiver_hpsdr2_trx_duo` to the topmost directory on the SD card.
- Install the micro SD card in the TRX-duo board and connect the power.
- Install and run one of the openHPSDR Protocol 2 programs listed above.

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

Building `sdr_receiver_hpsdr2_trx_duo.bit`:

```bash
make NAME=sdr_receiver_hpsdr2_trx_duo bit
```

Building SD card image zip file:

```bash
source helpers/build-all.sh
```
