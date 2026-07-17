# Create port_slicer
cell port_slicer slice_0 {
  DIN_WIDTH 8 DIN_FROM 0 DIN_TO 0
}

# Create port_slicer
cell port_slicer slice_1 {
  DIN_WIDTH 96 DIN_FROM 79 DIN_TO 0
}

# Create axis_fifo
cell axis_fifo fifo_0 {
  S_AXIS_TDATA_WIDTH 32
  M_AXIS_TDATA_WIDTH 64
  WRITE_DEPTH 16384
} {
  aclk /pll_0/clk_out1
  aresetn slice_0/dout
}

# Create axis_pulse_generator
cell axis_pulse_generator gen_0 {} {
  S_AXIS fifo_0/M_AXIS
  aclk /pll_0/clk_out1
  aresetn slice_0/dout
}

# Create axis_zeroer
cell axis_zeroer zeroer_0 {
  AXIS_TDATA_WIDTH 16
} {
  S_AXIS gen_0/M_AXIS
  aclk /pll_0/clk_out1
}

# Create axis_iir_filter
cell axis_iir_filter iir_0 {
  AXIS_TDATA_WIDTH 16
} {
  S_AXIS zeroer_0/M_AXIS
  cfg_data slice_1/dout
  aclk /pll_0/clk_out1
  aresetn slice_0/dout
}
