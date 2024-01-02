#ifndef __CONFIG_RED_PITAYA_H
#define __CONFIG_RED_PITAYA_H

#include <configs/zynq-common.h>

//#undef CONFIG_SYS_I2C_EEPROM_ADDR_LEN
//#undef CONFIG_SYS_I2C_EEPROM_ADDR
//#undef CONFIG_SYS_EEPROM_PAGE_WRITE_BITS
//#undef CONFIG_SYS_EEPROM_SIZE

//#undef CONFIG_ENV_IS_NOWHERE
//#undef CONFIG_ENV_SIZE
//#undef CONFIG_ENV_OFFSET
#undef CONFIG_EXTRA_ENV_SETTINGS

//#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN		2
//#define CONFIG_SYS_I2C_EEPROM_ADDR		0x50
//#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	5
//#define CONFIG_SYS_EEPROM_SIZE			8192 /* Bytes */

//#define CONFIG_ENV_IS_IN_EEPROM
//#define CONFIG_ENV_SIZE		1024 /* Total Size of Environment Sector */
//#define CONFIG_ENV_OFFSET	(2048*3) /* WP area starts at last 1/4 of 8k eeprom */


#define CONFIG_EXTRA_ENV_SETTINGS \
    "arch=arm\0" \
    "board=zynq\0" \
    "board_name=zynq\0" \
    "bootm_low=0\0" \
    "bootm_size=20000000\0" \
    "cpu=armv7\0" \
    "modeboot=sdboot\0" \
    "soc=zynq\0" \
    "loadaddr=0x0\0" \
    "script_offset_f=fc0000\0" \
    "scriptaddr=0\0" \
    "bootcmd=run $modeboot\0" \
    "bootdelay=3\0" \
    "baudrate=115200\0" \
    "ipaddr=10.10.70.102\0" \
    "serverip=10.10.70.101\0" \
    "prod_date=01/01/13\0" \
    "kernel_image=uImage\0" \
    "ramdisk_image=uramdisk.image.gz\0" \
    "devicetree_image=devicetree.dtb\0" \
    "bitstream_image=system.bit.bin\0" \
    "loadbit_addr=0x100000\0" \
    "kernel_size=0x500000\0" \
    "devicetree_size=0x20000\0" \
    "ramdisk_size=0x5E0000\0" \
    "fdt_high=0x20000000\0" \
    "initrd_high=0x20000000\0" \
    "sdboot=echo Running script from SD... && mmcinfo && fatload mmc 0 0x2000000 u-boot.scr && source 0x2000000\0" \
    "ethaddr=88:88:88:88:88:88\0" \
    "nav_code=0\0" \
    "hw_rev=0\0" \
    "serial=0\0"
    

#endif // CONFIG_RED_PITAYA_H