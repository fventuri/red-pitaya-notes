
`timescale 1 ns / 1 ps

/*
 * wb_capture - Protocol-2 wideband snapshot gate.
 *
 * On each rising edge of `arm`, streams exactly NSAMPLES consecutive raw ADC
 * samples (one per aclk, ADC is free-running at the sample clock) to a BRAM
 * writer, packing two 16-bit samples into each 32-bit output word (low half =
 * first/even sample, high half = second/odd sample). After NSAMPLES it returns
 * to idle and holds the buffer frozen until `arm` is taken low and high again.
 *
 * The paired axis_bram_writer increments its address once per output word and
 * wraps at NSAMPLES/2, so every capture overwrites word i with samples
 * [2i, 2i+1] and the buffer is self-aligned (no per-arm address reset needed).
 *
 * Handshake with the server (no status readback): server pulses arm high,
 * waits > NSAMPLES/f_adc (131 us @ 125 MHz) for the snapshot to complete, reads
 * the BRAM, then drops arm to re-ready the gate. Kept deliberately tiny (a
 * counter + a few regs, no DSP) for the LUT-tight normal receiver.
 */
module wb_capture #
(
  parameter integer NSAMPLES = 16384   /* must be even */
)
(
  input  wire        aclk,
  input  wire        aresetn,
  input  wire        arm,              /* level from cfg; rising edge starts a capture */
  input  wire [15:0] adc,              /* raw ADC0 sample, valid every clock           */

  output reg  [31:0] m_axis_tdata,     /* {odd_sample, even_sample}                    */
  output reg         m_axis_tvalid
);
  localparam integer CNTW = $clog2(NSAMPLES);   /* 14 for 16384 */

  reg [CNTW-1:0] cnt;        /* sample index within a capture, 0..NSAMPLES-1 */
  reg            state;      /* 0 = idle/hold, 1 = capturing                 */
  reg            arm_d;      /* for rising-edge detect                       */
  reg [15:0]     even_s;     /* latched even (low-half) sample               */

  wire arm_rise = arm & ~arm_d;

  always @(posedge aclk)
  begin
    arm_d         <= arm;
    m_axis_tvalid <= 1'b0;

    if(~aresetn)
    begin
      state <= 1'b0;
      cnt   <= {CNTW{1'b0}};
    end
    else
    begin
      case(state)
      1'b0:                         /* idle: wait for a fresh arm rising edge */
      begin
        cnt <= {CNTW{1'b0}};
        if(arm_rise) state <= 1'b1;
      end
      1'b1:                         /* capturing: one ADC sample per clock */
      begin
        if(cnt[0] == 1'b0)
          even_s <= adc;                       /* even sample -> low half */
        else
        begin
          m_axis_tdata  <= {adc, even_s};      /* odd sample -> high half, emit word */
          m_axis_tvalid <= 1'b1;
        end
        if(cnt == NSAMPLES - 1) state <= 1'b0; /* NSAMPLES captured; require arm low->high */
        cnt <= cnt + 1'b1;
      end
      endcase
    end
  end
endmodule
