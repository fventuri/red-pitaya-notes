source /opt/Xilinx/Vitis/2024.1/settings64.sh

JOBS=`nproc 2> /dev/null || echo 1`

make -j $JOBS cores

make NAME=led_blinker all

PRJS="sdr_receiver_trx_duo sdr_receiver_hpsdr_trx_duo sdr_receiver_wide_trx_duo sdr_transceiver_trx_duo sdr_transceiver_ft8_trx_duo sdr_transceiver_hpsdr_trx_duo sdr_transceiver_wide_trx_duo sdr_transceiver_wspr_trx_duo mcpha pulsed_nmr scanner vna_trx_duo playground template"

printf "%s\n" $PRJS | xargs -n 1 -P $JOBS -I {} make NAME={} bit

sudo sh scripts/alpine.sh
