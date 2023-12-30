#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

if grep -q '' $apps_dir/sdr_transceiver_wspr_trx_duo/decode-wspr.sh
then
  mount -o rw,remount /media/mmcblk0p1
  dos2unix $apps_dir/sdr_transceiver_wspr_trx_duo/decode-wspr.sh
  mount -o ro,remount /media/mmcblk0p1
fi

rm -rf /dev/shm/*

cat $apps_dir/sdr_transceiver_wspr_trx_duo/sdr_transceiver_wspr_trx_duo.bit > /dev/xdevcfg

ln -sf $apps_dir/sdr_transceiver_wspr_trx_duo/wspr.cron /etc/cron.d/wspr

service dcron restart
