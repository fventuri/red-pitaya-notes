`timescale 1 ns / 1 ps

// Time-shared multichannel complex mixer bank (halves the mixer DSP count).
//
// Replaces the 2*N_DDC parallel `dsp48` mixers + the `xlconcat` product bus of the
// narrow receiver. Each DDC needs two products from the SAME selected ADC sample:
//     I = round(cos * adc)   (channel 2*d)
//     Q = round(sin * adc)   (channel 2*d+1)
// The input rate is 125 Msps and the clock is 250 MHz => 2 cycles per input sample,
// so ONE DSP per DDC computes both products by time-multiplexing its A input
// (cos on the even cycle, sin on the odd cycle). DSP count: 2*N_DDC -> N_DDC.
//
// Bit-exactness: the multiply/round is the SAME `dsp48` primitive the parallel path
// used (A_WIDTH 24, B_WIDTH 16, P_WIDTH 24, convergent rounding, SHIFT=15), so each
// product is identical bit-for-bit to the old mixer.
//
// Drop-in timing: cic_ts_bank's carry-split integrator requires in_data to be stable
// across each stream's 2-cycle add window, i.e. the product bus may change ONLY at
// input-sample (period) boundaries, with ALL channels updating together. The parallel
// dsp48 mixers did that naturally (products change once per 125 MHz period, PREG-phase).
// Here I and Q emerge on consecutive cycles, so they are captured into per-DDC holding
// regs and then republished together, once per period, into `out_data` -- reproducing
// the parallel bus's "all channels change at one phase per period" behaviour. The extra
// pipeline (vs the parallel path) is a whole number of periods, so the sub-sample
// decimation phase is unchanged (irrelevant to the receiver; verified in sim).
//
// The republish register also gives the mixer->CIC hand-off its own pipeline cycle:
// the long route from the DSP column to the integrator lanes now ends at a movable
// fabric flop instead of feeding the 64-bit stage-0 add in the same cycle -- which is
// what lets clk_out2 (250 MHz) close.
//
// out_data layout matches the old xlconcat: {ch(2*N_DDC-1) ... ch1, ch0}, ch0 in LSBs,
// P_W bits each, channel 2*d = I(d), channel 2*d+1 = Q(d).

module mixer_ts_bank #
(
  parameter integer N_DDC = 16,   // number of DDCs (product bus has 2*N_DDC channels)
  parameter integer NCO_W = 24,   // cos/sin sample width
  parameter integer ADC_W = 16,   // selected ADC width
  parameter integer P_W   = 24    // product width (matches dsp48 P_WIDTH)
)
(
  input  wire                      aclk,     // 250 MHz DSP clock (clk_out2)
  input  wire                      aresetn,  // from rst_1 (same domain as cic_ts_bank)

  input  wire [N_DDC*48-1:0]       nco,      // per DDC {sin[23:0], cos[23:0]}, 125 MHz-stable
  input  wire [N_DDC*ADC_W-1:0]    adc,      // per DDC selected ADC sample, 125 MHz-stable

  output wire [2*N_DDC*P_W-1:0]    out_data  // channel-interleaved product bus, period-stable
);

  genvar d;

  // ---------------------------------------------------------------------------
  // Parity: ph toggles every cycle. ph==0 selects cos (-> I), ph==1 selects sin
  // (-> Q). Counting from reset keeps the bank phase-locked to cic_ts_bank (same
  // aresetn, same clock) so the republished bus lands on cic's period boundary.
  // ---------------------------------------------------------------------------
  reg ph_reg;
  always @(posedge aclk)
    if(~aresetn) ph_reg <= 1'b0;
    else         ph_reg <= ~ph_reg;

  // dsp48 latency is AREG + MREG + PREG = 3 cycles. Track which product (I or Q) is
  // emerging by delaying the select bit through the same 3 stages.
  localparam integer MUL_LAT = 3;
  reg [MUL_LAT-1:0] slot_pipe;   // slot_pipe[MUL_LAT-1] aligns with the DSP output
  always @(posedge aclk)
    if(~aresetn) slot_pipe <= {MUL_LAT{1'b0}};
    else         slot_pipe <= {slot_pipe[MUL_LAT-2:0], ph_reg};
  wire out_is_q = slot_pipe[MUL_LAT-1];   // 0 -> product is I(cos), 1 -> product is Q(sin)

  // Republish strobe: one cycle after a Q product has been captured, both I and Q of
  // that DDC are fresh, so latch the whole bus. This makes out_data change once per
  // period, all channels together.
  reg republish;
  always @(posedge aclk)
    if(~aresetn) republish <= 1'b0;
    else         republish <= out_is_q;

  generate
  for(d=0; d<N_DDC; d=d+1) begin : mix
    wire [NCO_W-1:0] cos_d = nco[d*48        +: NCO_W];
    wire [NCO_W-1:0] sin_d = nco[d*48 + 24   +: NCO_W];
    wire [ADC_W-1:0] adc_d = adc[d*ADC_W     +: ADC_W];

    // Per-lane replica of the parity bit that drives THIS lane's cos/sin mux. It is
    // value-identical to the global ph_reg every cycle (same clock, same reset, same
    // toggle from 0), but keeping a private copy per DDC lets the placer put the mux
    // select next to each lane's DSP instead of routing one high-fanout ph_reg net out
    // to all the spread-out DSP columns (that route was the last ~-0.02 ns clk_out2
    // path). `keep` stops equivalent-register removal from merging them back.
    (* keep = "true" *) reg ph_d;
    always @(posedge aclk)
      if(~aresetn) ph_d <= 1'b0;
      else         ph_d <= ~ph_d;

    // time-multiplexed A input: cos on ph==0, sin on ph==1 (registered inside dsp48)
    wire [NCO_W-1:0] a_mux = ph_d ? sin_d : cos_d;

    wire [P_W-1:0] prod_d;
    dsp48 #(
      .A_WIDTH(NCO_W), .B_WIDTH(ADC_W), .P_WIDTH(P_W)
    ) mult (
      .CLK(aclk), .A(a_mux), .B(adc_d), .P(prod_d)
    );

    // capture each product into its I/Q holding reg as it emerges
    reg [P_W-1:0] i_hold, q_hold;
    always @(posedge aclk) begin
      if(~aresetn) begin i_hold <= {P_W{1'b0}}; q_hold <= {P_W{1'b0}}; end
      else if(out_is_q) q_hold <= prod_d;
      else              i_hold <= prod_d;
    end

    // period-stable published outputs (all DDCs republish on the same strobe). No reset:
    // these are reloaded from upstream within one period of reset release, and the
    // downstream CIC does not consume valid output until ~1000 cycles after reset (its
    // decimation counter + prev-RAM clear). Dropping the reset removes a high-fanout,
    // long-route reset net from these registers' R pins (it was the binding clk_out2
    // path once the integrator moved to clk_out1).
    reg [P_W-1:0] i_out, q_out;
    always @(posedge aclk)
      if(republish) begin i_out <= i_hold; q_out <= q_hold; end

    // One extra register so the total mixer latency is an EVEN number of cycles
    // (a whole 2-cycle period) relative to the old parallel path -> the product bus
    // lands on the same cic period phase (verified in sim: K=3 odd -> K=4 even).
    reg [P_W-1:0] i_pub, q_pub;
    always @(posedge aclk) begin
      i_pub <= i_out;
      q_pub <= q_out;
    end

    assign out_data[(2*d)  *P_W +: P_W] = i_pub;   // channel 2*d   = I
    assign out_data[(2*d+1)*P_W +: P_W] = q_pub;   // channel 2*d+1 = Q
  end
  endgenerate

endmodule
