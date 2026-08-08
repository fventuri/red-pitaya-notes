source /opt/Xilinx/2026.1/Vitis/settings64.sh

JOBS=`nproc 2> /dev/null || echo 1`

make NAME=led_blinker all

#PRJS="sdr_receiver sdr_receiver_hpsdr sdr_receiver_wide sdr_transceiver sdr_transceiver_ft8 sdr_transceiver_hpsdr sdr_transceiver_wide sdr_transceiver_wspr mcpha pulsed_nmr scanner vna playground template"
#
#printf "%s\n" $PRJS | xargs -n 1 -P $JOBS -I {} make NAME={} bit
#
# 122M88 app set builds for the Zynq 7020 (SDRlab-style 122.88 MHz board)
PRJS="led_blinker_122M88 sdr_receiver_122M88 sdr_receiver_hpsdr_122M88 sdr_receiver_wide_122M88 sdr_transceiver_122M88 sdr_transceiver_ft8_122M88 sdr_transceiver_hpsdr_122M88 sdr_transceiver_wspr_122M88 pulsed_nmr_122M88 vna_122M88"

printf "%s\n" $PRJS | xargs -n 1 -P $JOBS -I {} make NAME={} PART=xc7z020clg400-1 bit

# 125M app set builds for the Zynq 7010 (STEMlab-style 125 MHz board = original TRX-duo)
# all 16 _125M projects (formerly _trx_duo), incl. the four hpsdr2 receivers, plus playground
PRJS="mcpha_125M playground pulsed_nmr_125M scanner_125M sdr_receiver_125M sdr_receiver_hpsdr_125M sdr_receiver_hpsdr2_125M sdr_receiver_hpsdr2_narrow_125M sdr_receiver_hpsdr2_wide_125M sdr_receiver_hpsdr2_extrawide_125M sdr_receiver_wide_125M sdr_transceiver_125M sdr_transceiver_ft8_125M sdr_transceiver_hpsdr_125M sdr_transceiver_wide_125M sdr_transceiver_wspr_125M vna_125M"

printf "%s\n" $PRJS | xargs -n 1 -P $JOBS -I {} make NAME={} PART=xc7z010clg400-1 bit

sudo sh scripts/alpine.sh
