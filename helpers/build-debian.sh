source /opt/Xilinx/Vitis/2024.2/settings64.sh

DATE=`date +%Y%m%d`

make NAME=led_blinker all

sudo sh scripts/image.sh scripts/debian.sh red-pitaya-debian-12.8-armhf-$DATE.img 1024
zip red-pitaya-debian-12.8-armhf-$DATE.zip red-pitaya-debian-12.8-armhf-$DATE.img
