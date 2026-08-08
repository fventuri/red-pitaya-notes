#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

. $apps_dir/stop.sh

cat $apps_dir/scanner_125M/scanner_125M.bit > /dev/xdevcfg

$apps_dir/scanner_125M/scanner &
