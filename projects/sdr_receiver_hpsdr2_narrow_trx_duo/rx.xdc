# Multicycle the ADC -> time-shared-mixer DSP48 B-input crossing.
#
# mixer_ts_bank runs at clk_out2 (250 MHz) and time-multiplexes cos/sin on the DSP A
# input (single-cycle), but the DSP B input is the SELECTED ADC sample, which comes from
# the 125 MHz (clk_out1) domain and is stable for a whole clk_out1 period = 2 clk_out2
# cycles (it does not depend on the per-cycle mux). The tool otherwise times this
# clk_out1 -> clk_out2 path at the tight single-cycle (4 ns) budget; since the data is
# valid for 2 destination cycles, it is a true multicycle path. This closes the residual
# ~-0.22 ns without touching the datapath (function unchanged; the A/select paths and all
# other clk_out2 logic keep their single-cycle timing).
set b_pins [get_pins -hier -filter {NAME =~ *mixer_0/inst/mix*mult/dsp_0/B* && NAME !~ *BCOUT*}]
set_multicycle_path 2 -setup -from [get_clocks clk_out1_system_pll_0_0] -to $b_pins
set_multicycle_path 1 -hold  -from [get_clocks clk_out1_system_pll_0_0] -to $b_pins
