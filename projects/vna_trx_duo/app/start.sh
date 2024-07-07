#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

cat $apps_dir/vna_trx_duo/vna_trx_duo.bit > /dev/xdevcfg

$apps_dir/vna_trx_duo/vna &
