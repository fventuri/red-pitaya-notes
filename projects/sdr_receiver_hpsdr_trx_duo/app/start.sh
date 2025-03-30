#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

cat $apps_dir/sdr_receiver_hpsdr_trx_duo/sdr_receiver_hpsdr_trx_duo.bit > /dev/xdevcfg

$apps_dir/sdr_receiver_hpsdr_trx_duo/sdr-receiver-hpsdr 1 1 1 1 1 1 1 1 &
