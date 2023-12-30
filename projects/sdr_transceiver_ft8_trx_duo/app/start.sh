#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

if grep -q '' $apps_dir/sdr_transceiver_ft8_trx_duo/upload-ft8.sh
then
  mount -o rw,remount /media/mmcblk0p1
  dos2unix $apps_dir/sdr_transceiver_ft8_trx_duo/upload-ft8.sh
  mount -o ro,remount /media/mmcblk0p1
fi

rm -rf /dev/shm/*

cat $apps_dir/sdr_transceiver_ft8_trx_duo/sdr_transceiver_ft8_trx_duo.bit > /dev/xdevcfg

ln -sf $apps_dir/sdr_transceiver_ft8_trx_duo/ft8.cron /etc/cron.d/ft8

service dcron restart
