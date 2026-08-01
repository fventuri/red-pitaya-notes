
`timescale 1 ns / 1 ps

// Phase-truncation NCO (0 DSP), optional phase dithering.
//
// Functionally a drop-in replacement for cores/dds.v but WITHOUT the 3 DSP48E1
// Taylor-interpolation slices. The phase accumulator addresses the same shared
// quarter-wave sine ROM (dds.mem, 2048 x 23-bit) with the same quadrant fold /
// sign logic; the raw LUT sample is emitted directly instead of being refined.
//
// Cost per instance: 0 DSP, 1 BRAM (dual-port quarter-wave ROM), a 32-bit phase
// accumulator + a little fold/sign logic in LUTs.
//
// SFDR is set by the phase truncation. The quarter-wave LUT uses an 11-bit address +
// 2 quadrant bits = 13 effective phase bits, so the intrinsic worst-case discrete spur
// is ~ -76..-78 dBc (measured in xsim; better than the -66 dBc a naive 11-bit count
// gives). See PLAN_NARROW_48_MAXDDC 2b and docs/REPLY_HENNING_dds_phase_sfdr.md.
//
// DITHER = "TRUE" (default) adds a 19-bit maximal-length LFSR to the accumulator's
// truncated low bits [18:0] before the LUT address/quadrant are derived. The accumulator
// itself is NOT dithered, so the output frequency is exact; only the truncation is
// randomized. This converts the discrete phase-truncation spurs into a low broadband
// noise floor (~ -119 dBc/bin in xsim; ~ -100 dBc integrated in-band -> negligible),
// so a strong neighbouring signal folds in as diffuse noise rather than a discrete
// in-band birdie (the Henning reciprocal-mixing concern). Cost: ~a few dozen LUTs, 0 DSP,
// no extra BRAM. Set DITHER = "FALSE" to restore the plain-truncation behaviour.
//
// dout layout matches dds.v: {sin[23:0], cos[23:0]} (cos in the low half).

module dds_phase #
(
  parameter NEGATIVE_SINE = "FALSE",
  parameter DITHER        = "TRUE"
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

  // Phase used to derive the LUT address/quadrant: the exact accumulator, optionally with
  // an LFSR dither added to its truncated low bits (see DITHER above).
  wire [31:0] int_phase_wire;

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

  generate
    if(DITHER == "TRUE")
    begin : DITH
      // 19-bit maximal-length Fibonacci LFSR (taps 19,18,17,14), added to bits [18:0] so
      // the carry into the LUT address (bit 19) is randomized (~1-LSB uniform dither).
      reg [18:0] lfsr_reg;
      wire lfsr_fb = lfsr_reg[18] ^ lfsr_reg[17] ^ lfsr_reg[16] ^ lfsr_reg[13];
      always @(posedge aclk)
        if(~aresetn) lfsr_reg <= 19'h1;
        else         lfsr_reg <= {lfsr_reg[17:0], lfsr_fb};
      assign int_phase_wire = int_cntr_reg + {13'd0, lfsr_reg};
    end
    else
    begin : NODITH
      assign int_phase_wire = int_cntr_reg;
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
      int_addr_reg <= int_phase_wire[29:19];
      int_sign_reg[0] <= {int_phase_wire[31], int_phase_wire[30]};
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
