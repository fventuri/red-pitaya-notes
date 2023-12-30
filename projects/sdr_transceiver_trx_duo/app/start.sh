#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

cat $apps_dir/sdr_transceiver_trx_duo/sdr_transceiver_trx_duo.bit > /dev/xdevcfg

$apps_dir/sdr_transceiver_trx_duo/sdr-transceiver 1 &
$apps_dir/sdr_transceiver_trx_duo/sdr-transceiver 2 &
