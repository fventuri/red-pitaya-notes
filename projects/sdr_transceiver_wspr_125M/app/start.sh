#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

# Refuse to reprogram the PL if an ACP/DMA design was loaded this boot (it would hang).
# Only the HPSDR2 receivers set this flag; any app that reprograms the PL afterward hangs.
if [ -f /tmp/needs-reboot ]; then
  echo "ERROR: an ACP/DMA FPGA design was already loaded this boot; reprogramming the" >&2
  echo "PL now would hang the board. Reboot before starting this app." >&2
  exit 1
fi

. $apps_dir/stop.sh

if grep -q '
' $apps_dir/sdr_transceiver_wspr_125M/decode-wspr.sh
then
  mount -o rw,remount /media/mmcblk0p1
  dos2unix $apps_dir/sdr_transceiver_wspr_125M/decode-wspr.sh
  mount -o ro,remount /media/mmcblk0p1
fi

rm -rf /dev/shm/*

cat $apps_dir/sdr_transceiver_wspr_125M/sdr_transceiver_wspr_125M.bit > /dev/xdevcfg

ln -sf $apps_dir/sdr_transceiver_wspr_125M/wspr.cron /etc/cron.d/wspr

service dcron restart
