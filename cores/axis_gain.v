
`timescale 1 ns / 1 ps

// Constant fixed-point gain with saturation for an AXIS stream, as a 1-deep
// registered AXIS stage (proper skid handshake, backpressure-safe). The multiply is
// steered into a DSP48 (use_dsp) to keep it off the congested 7010 fabric.
// out = sat( (signed(low DATA_WIDTH bits) * GAIN_NUM) >>> GAIN_SHIFT ), 1-cycle latency.
// GAIN_NUM/GAIN_SHIFT default 7332/1024 = 7.160x = +17.1 dB.
module axis_gain #
(
  parameter integer AXIS_TDATA_WIDTH = 32,
  parameter integer DATA_WIDTH       = 24,
  parameter integer GAIN_NUM         = 7332,
  parameter integer GAIN_SHIFT       = 10
)
(
  // System signals
  input  wire                        aclk,
  input  wire                        aresetn,

  // Slave side
  output wire                        s_axis_tready,
  input  wire [AXIS_TDATA_WIDTH-1:0] s_axis_tdata,
  input  wire                        s_axis_tvalid,
  input  wire                        s_axis_tlast,

  // Master side
  input  wire                        m_axis_tready,
  output wire [AXIS_TDATA_WIDTH-1:0] m_axis_tdata,
  output wire                        m_axis_tvalid,
  output wire                        m_axis_tlast
);

  localparam integer PW = DATA_WIDTH + 18;   // product width (25x18 signed DSP48)

  wire signed [DATA_WIDTH-1:0]  in     = s_axis_tdata[DATA_WIDTH-1:0];
  (* use_dsp = "yes" *)
  wire signed [PW-1:0]          prod   = $signed(in) * $signed(GAIN_NUM[17:0]);
  wire signed [PW-1:0]          scaled = prod >>> GAIN_SHIFT;

  // cheap saturation to DATA_WIDTH signed: the bits above bit (DATA_WIDTH-1) must all
  // equal the sign bit, else clamp to +/- full scale.
  wire signed [DATA_WIDTH-1:0] maxv = {1'b0, {(DATA_WIDTH-1){1'b1}}};
  wire signed [DATA_WIDTH-1:0] minv = {1'b1, {(DATA_WIDTH-1){1'b0}}};
  wire ovf_hi = ~scaled[PW-1] &  (|scaled[PW-2:DATA_WIDTH-1]);
  wire ovf_lo =  scaled[PW-1] & ~(&scaled[PW-2:DATA_WIDTH-1]);
  wire signed [DATA_WIDTH-1:0] sat = ovf_hi ? maxv : ovf_lo ? minv : scaled[DATA_WIDTH-1:0];

  reg [AXIS_TDATA_WIDTH-1:0] data_reg;
  reg                        last_reg;
  reg                        valid_reg;

  wire load = s_axis_tvalid & s_axis_tready;

  always @(posedge aclk)
  begin
    if(~aresetn)
      valid_reg <= 1'b0;
    else if(load)
      valid_reg <= 1'b1;
    else if(m_axis_tready)
      valid_reg <= 1'b0;

    if(load)
    begin
      data_reg <= {{(AXIS_TDATA_WIDTH-DATA_WIDTH){sat[DATA_WIDTH-1]}}, sat};
      last_reg <= s_axis_tlast;
    end
  end

  assign s_axis_tready = m_axis_tready | ~valid_reg;
  assign m_axis_tdata  = data_reg;
  assign m_axis_tvalid = valid_reg;
  assign m_axis_tlast  = last_reg;

endmodule
