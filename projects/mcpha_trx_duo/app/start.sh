#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

source $apps_dir/stop.sh

cat $apps_dir/mcpha_trx_duo/mcpha_trx_duo.bit > /dev/xdevcfg

$apps_dir/mcpha_trx_duo/mcpha-server &
$apps_dir/mcpha_trx_duo/pha-server &
