
`timescale 1 ns / 1 ps

// Phase-truncation NCO (0 DSP).
//
// Functionally a drop-in replacement for cores/dds.v but WITHOUT the 3 DSP48E1
// Taylor-interpolation slices. The phase accumulator addresses the same shared
// quarter-wave sine ROM (dds.mem, 2048 x 23-bit) with the same quadrant fold /
// sign logic; the raw LUT sample is emitted directly instead of being refined.
//
// Cost per instance: 0 DSP, 1 BRAM (dual-port quarter-wave ROM), a 32-bit phase
// accumulator + a little fold/sign logic in LUTs.
//
// SFDR is set by the 11-bit phase truncation (~66 dBc), vs. ~90 dBc for the
// interpolated dds.v. That trade is the whole point of the narrow project: it
// frees the 3 DSP/DDC that otherwise cap the DDC count. See PLAN_NARROW_48_MAXDDC
// section 2b (option A) - the on-air SFDR is the gate that decides whether this
// suffices or the interpolated NCO must come back.
//
// dout layout matches dds.v: {sin[23:0], cos[23:0]} (cos in the low half).

module dds_phase #
(
  parameter NEGATIVE_SINE = "FALSE"
)
(
  input  wire        aclk,
  input  wire        aresetn,

  input  wire [31:0] pinc,

  output wire [47:0] dout
);

  reg [31:0] int_cntr_reg;
  reg [23:0] int_cos_reg, int_sin_reg;
  reg [10:0] int_addr_reg;
  reg [1:0] int_sign_reg [2:0];

  wire [31:0] int_cntr_wire;
  wire [29:0] int_cos_wire, int_sin_wire;
  wire [22:0] int_lut_wire [1:0];

  // Sign-folded quadrant reconstruction, identical to dds.v.
  assign int_cos_wire = int_sign_reg[2][0] ? {7'h7f, -int_lut_wire[0]} : {7'h00, int_lut_wire[0]};
  assign int_sin_wire = int_sign_reg[2][1] ? {7'h7f, -int_lut_wire[1]} : {7'h00, int_lut_wire[1]};

  generate
    if(NEGATIVE_SINE == "TRUE")
    begin : NEGATIVE
      assign int_cntr_wire = int_cntr_reg - pinc;
    end
    else
    begin : POSITIVE
      assign int_cntr_wire = int_cntr_reg + pinc;
    end
  endgenerate

  always @(posedge aclk)
  begin
    if(~aresetn)
    begin
      int_cntr_reg <= 32'd0;
      int_cos_reg <= 24'd0;
      int_sin_reg <= 24'd0;
      int_addr_reg <= 11'd0;
      int_sign_reg[0] <= 2'd0;
      int_sign_reg[1] <= 2'd0;
      int_sign_reg[2] <= 2'd0;
    end
    else
    begin
      int_cntr_reg <= int_cntr_wire;
      int_cos_reg <= int_cos_wire[23:0];
      int_sin_reg <= int_sin_wire[23:0];
      int_addr_reg <= int_cntr_reg[29:19];
      int_sign_reg[0] <= {int_cntr_reg[31], int_cntr_reg[30]};
      int_sign_reg[1] <= {int_sign_reg[0][1], int_sign_reg[0][1] ^ int_sign_reg[0][0]};
      int_sign_reg[2] <= int_sign_reg[1];
    end
  end

  xpm_memory_dprom #(
    .MEMORY_PRIMITIVE("block"),
    .MEMORY_SIZE(47104),
    .ADDR_WIDTH_A(11),
    .ADDR_WIDTH_B(11),
    .READ_DATA_WIDTH_A(23),
    .READ_DATA_WIDTH_B(23),
    .READ_LATENCY_A(2),
    .READ_LATENCY_B(2),
    .MEMORY_INIT_PARAM(""),
    .MEMORY_INIT_FILE("dds.mem")
  ) rom_0 (
    .clka(aclk),
    .clkb(aclk),
    .rsta(1'b0),
    .rstb(1'b0),
    .ena(1'b1),
    .enb(1'b1),
    .regcea(1'b1),
    .regceb(1'b1),
    .addra(int_sign_reg[0][0] ? ~int_addr_reg : int_addr_reg),
    .addrb(int_sign_reg[0][0] ? int_addr_reg : ~int_addr_reg),
    .douta(int_lut_wire[0]),
    .doutb(int_lut_wire[1])
  );

  assign dout = {int_sin_reg, int_cos_reg};

endmodule
