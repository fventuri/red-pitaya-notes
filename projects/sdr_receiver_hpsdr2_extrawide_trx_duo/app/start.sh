#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

# The ACP/DMA data path hangs if the PL is reprogrammed after the ACP port has been
# active. Once one of the HPSDR2 receivers has loaded its bitstream this boot, refuse
# to program the PL again until a reboot. /tmp is tmpfs, so the flag clears on reboot.
if [ -f /tmp/needs-reboot ]; then
  echo "ERROR: an ACP/DMA FPGA design was already loaded this boot; reprogramming the" >&2
  echo "PL now would hang the board. Reboot before starting this receiver." >&2
  exit 1
fi

. $apps_dir/stop.sh

# create the reboot guard right before loading the bitstream
touch /tmp/needs-reboot

cat $apps_dir/sdr_receiver_hpsdr2_extrawide_trx_duo/sdr_receiver_hpsdr2_extrawide_trx_duo.bit > /dev/xdevcfg

$apps_dir/sdr_receiver_hpsdr2_extrawide_trx_duo/sdr-receiver-hpsdr2 &
