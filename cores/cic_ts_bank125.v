
`timescale 1 ns / 1 ps

// Un-folded multichannel CIC decimator running in the 125 MHz domain (clk_out1).
//
// Same math and output slice as cic_ts_bank.v, but the integrators are NOT time-
// shared: each of the N_CH streams gets its own single-stream integrator that does
// ONE full-width add per stage per cycle. Because the input rate is 125 Msps and the
// clock is now 125 MHz (one sample per cycle), a stage add has the whole 8 ns period
// to close, so the ACC_W-bit accumulate is single-cycle (no carry-split, no parity).
// This removes the 250 MHz full-rate-integrator timing wall that the folded core hits
// on the xc7z010 -1 part, at the cost of ~2x the integrator adders (area for timing).
//
// Clocking / CDC:
//   * aclk is clk_out1 (125 MHz). The mixer (mixer_ts_bank) runs at clk_out2 (250 MHz)
//     and publishes a PERIOD-STABLE product bus (all channels change once per 125 MHz
//     input period). clk_out1 and clk_out2 come from the same PLL (aligned, 2:1), and
//     both sides move one sample per period, so sampling in_data once per clk_out1
//     cycle captures each period's products exactly once (synchronous, rate-matched).
//     in_data is registered once here (ind_reg) to isolate that crossing to a single
//     flop and give the stage-0 add a clean 125 MHz start.
//   * The output M_AXIS is in the 125 MHz domain; wire it to fir_0 (250 MHz) through an
//     axis_clock_converter (done in rx.tcl), not directly.
//
// The comb engine + AXIS output section below is reused verbatim from cic_ts_bank.v
// (low rate, ample headroom at 125 MHz: ~R cycles between decimations vs the
// N_CH*STAGES*4 it needs). The output bit-slice DOUT = comb_w[SLICE_HI -: DOUT_W] is
// unchanged, so the gain matches cic_compiler exactly (pinned by the DC sim). As with
// the folded core, the absolute decimation phase may differ by a constant sub-sample
// offset, which is irrelevant to the receiver (identical magnitude/level).

module cic_ts_bank125 #
(
  parameter integer N_CH     = 32,   // number of streams (= 2 * n_ddc), must be even
  parameter integer DIN_W    = 24,   // signed mixer product width
  parameter integer DOUT_W   = 32,   // CIC output width (truncated)
  parameter integer STAGES   = 4,
  parameter integer R        = 1000, // decimation factor (M=1)
  parameter integer ACC_W    = 64,   // DIN_W + ceil(STAGES*log2(R*M))
  parameter integer SLICE_HI = 63    // MSB of the DOUT_W-wide output slice
)
(
  input  wire                   aclk,     // 125 MHz clock (clk_out1)
  input  wire                   aresetn,  // from rst_1

  input  wire [N_CH*DIN_W-1:0]  in_data,  // period-stable mixer bus, ch0 in LSBs

  output wire [DOUT_W-1:0]      m_axis_tdata,
  output wire                   m_axis_tvalid,
  input  wire                   m_axis_tready
);

  localparam integer PREV_N  = N_CH * STAGES;                 // comb prev-state entries
  localparam integer PADDR_W = (PREV_N <= 1) ? 1 : $clog2(PREV_N);
  localparam integer SCNT_W  = (N_CH  <= 1) ? 1 : $clog2(N_CH);
  localparam integer PCNT_W  = (R     <= 1) ? 1 : $clog2(R);

  genvar L, st;

  // ---------------------------------------------------------------------------
  // Control: one input sample per cycle. pcnt counts 0..R-1; decimate_now marks the
  // period whose integrated result is captured; comb_start fires one cycle later, when
  // the cap[] registers have settled.
  // ---------------------------------------------------------------------------
  reg [PCNT_W-1:0]  pcnt_reg;
  reg               comb_start_reg;

  wire decimate_now = (pcnt_reg == R-1);

  always @(posedge aclk)
  begin
    if(~aresetn)
    begin
      pcnt_reg       <= {PCNT_W{1'b0}};
      comb_start_reg <= 1'b0;
    end
    else
    begin
      pcnt_reg       <= decimate_now ? {PCNT_W{1'b0}} : (pcnt_reg + 1'b1);
      comb_start_reg <= decimate_now;
    end
  end

  // register the cross-domain (250 MHz mixer -> 125 MHz) input bus once
  reg [N_CH*DIN_W-1:0] ind_reg;
  always @(posedge aclk)
    if(~aresetn) ind_reg <= {N_CH*DIN_W{1'b0}};
    else         ind_reg <= in_data;

  // ---------------------------------------------------------------------------
  // Integrator lanes (un-folded, full rate at 125 MHz, flip-flop state). One lane per
  // stream, STAGES accumulators, single-cycle full-width adds. cap[] holds the last-
  // stage accumulators sampled at the decimation instant (all channels, same cycle).
  // ---------------------------------------------------------------------------
  (* ram_style = "registers" *) reg [ACC_W-1:0] cap [0:N_CH-1];

  generate
  for(L=0; L<N_CH; L=L+1) begin : lane
    reg [ACC_W-1:0] acc [0:STAGES-1];

    integer ii;

    wire [DIN_W-1:0] x_raw = ind_reg[L*DIN_W +: DIN_W];
    wire [ACC_W-1:0] x_ext = {{(ACC_W-DIN_W){x_raw[DIN_W-1]}}, x_raw};

    // Pipelined integrator cascade: each stage adds the OLD previous-stage accumulator
    // (non-blocking RHS -> previous-cycle values), matching cic_ts_bank's structure, so
    // the gain/response is identical (verified vs cic_compiler; only a constant latency
    // differs). cap captures the last stage's new value at the decimation instant.
    always @(posedge aclk) begin
      if(~aresetn) begin
        for(ii=0; ii<STAGES; ii=ii+1) acc[ii] <= {ACC_W{1'b0}};
      end
      else begin
        acc[0] <= acc[0] + x_ext;
        for(ii=1; ii<STAGES; ii=ii+1)
          acc[ii] <= acc[ii] + acc[ii-1];
        if(decimate_now)
          cap[L] <= (STAGES == 1) ? (acc[0] + x_ext)
                                  : (acc[STAGES-1] + acc[STAGES-2]);
      end
    end
  end
  endgenerate

  // ---------------------------------------------------------------------------
  // Shared comb engine (reused verbatim from cic_ts_bank.v). Sequentially, for each
  // stream s and stage st:  w = v - prev[s][st];  prev[s][st] <= v;  v <= w.
  // prev[][] lives in BRAM (simple dual-port, read latency 1). A startup CLEAR pass
  // zeroes all prev entries before the first decimation.
  // ---------------------------------------------------------------------------
  localparam ST_RESET = 2'd0,   // clear prev RAM
             ST_IDLE  = 2'd1,   // wait for comb_start
             ST_RUN   = 2'd2,   // sequential comb over all streams
             ST_OUT   = 2'd3;   // stream results out over m_axis

  localparam integer LOW = ACC_W/2;        // subtract split point

  reg [1:0]         state_reg;
  reg [1:0]         ph_reg;                // sub-cycle within a (s,st) step (0..3)
  reg [SCNT_W-1:0]  s_reg;                 // current stream
  reg [3:0]         st_reg;                // current comb stage
  reg [PADDR_W-1:0] clr_addr_reg;          // clear-sweep address
  reg [ACC_W-1:0]   v_reg;                 // running cascade value
  reg [ACC_W-1:0]   prev_r_reg;            // registered prev[] BRAM read (isolates BRAM)
  reg [LOW-1:0]     diff_lo_reg;           // pipelined low half of v - prev
  reg               borrow_reg;            // borrow from the low half

  (* ram_style = "registers" *) reg [DOUT_W-1:0] out_buf [0:N_CH-1];

  // prev RAM interface
  reg               prev_we;
  reg [PADDR_W-1:0] prev_waddr;
  reg [ACC_W-1:0]   prev_wdata;
  wire [ACC_W-1:0]  prev_rdata;

  wire [PADDR_W-1:0] step_addr = s_reg*STAGES + st_reg;   // addr(s,st)

  // subtract split into two halves on separate cycles (kept from the 250 MHz core; at
  // 125 MHz it would close single-cycle too, but the pipelined form is harmless and the
  // comb has ~10x cycle headroom). Operands are the registered v_reg and prev_r_reg.
  wire [LOW-1:0]        sub_lo     = v_reg[LOW-1:0] - prev_r_reg[LOW-1:0];
  wire                  sub_borrow = (v_reg[LOW-1:0] < prev_r_reg[LOW-1:0]);
  wire [ACC_W-LOW-1:0]  sub_hi     = v_reg[ACC_W-1:LOW] - prev_r_reg[ACC_W-1:LOW] - borrow_reg;
  wire [ACC_W-1:0]      comb_w     = {sub_hi, diff_lo_reg};   // valid in phase 3

  // output stream
  reg [DOUT_W-1:0]  m_tdata_reg;
  reg               m_tvalid_reg;
  reg [SCNT_W-1:0]  oc_reg;
  reg               out_pending_reg;

  assign m_axis_tdata  = m_tdata_reg;
  assign m_axis_tvalid = m_tvalid_reg;

  always @(posedge aclk) begin
    if(~aresetn) begin
      state_reg       <= ST_RESET;
      ph_reg          <= 2'd0;
      s_reg           <= {SCNT_W{1'b0}};
      st_reg          <= 4'd0;
      clr_addr_reg    <= {PADDR_W{1'b0}};
      v_reg           <= {ACC_W{1'b0}};
      prev_r_reg      <= {ACC_W{1'b0}};
      diff_lo_reg     <= {LOW{1'b0}};
      borrow_reg      <= 1'b0;
      prev_we         <= 1'b0;
      prev_waddr      <= {PADDR_W{1'b0}};
      prev_wdata      <= {ACC_W{1'b0}};
      m_tvalid_reg    <= 1'b0;
      m_tdata_reg     <= {DOUT_W{1'b0}};
      oc_reg          <= {SCNT_W{1'b0}};
      out_pending_reg <= 1'b0;
    end
    else begin
      prev_we <= 1'b0;                      // default: no write this cycle

      case(state_reg)

      ST_RESET: begin
        prev_we    <= 1'b1;
        prev_waddr <= clr_addr_reg;
        prev_wdata <= {ACC_W{1'b0}};
        if(clr_addr_reg == PREV_N-1) begin
          clr_addr_reg <= {PADDR_W{1'b0}};
          state_reg    <= ST_IDLE;
        end
        else
          clr_addr_reg <= clr_addr_reg + 1'b1;
      end

      ST_IDLE: begin
        if(comb_start_reg) begin
          state_reg <= ST_RUN;
          s_reg     <= {SCNT_W{1'b0}};
          st_reg    <= 4'd0;
          ph_reg    <= 2'd0;
        end
      end

      ST_RUN: begin
        //   ph0: present addr(s,st); preload v for a stream's stage 0
        //   ph1: prev_rdata (BRAM) valid -> register it
        //   ph2: low half of v - prev_r_reg -> register
        //   ph3: high half -> comb_w; write prev<=v, v<=comb_w, out_buf, advance
        case(ph_reg)
        2'd0: begin
          if(st_reg == 4'd0)
            v_reg <= cap[s_reg];
          ph_reg <= 2'd1;
        end
        2'd1: begin
          prev_r_reg <= prev_rdata;
          ph_reg     <= 2'd2;
        end
        2'd2: begin
          diff_lo_reg <= sub_lo;
          borrow_reg  <= sub_borrow;
          ph_reg      <= 2'd3;
        end
        default: begin   // ph3
          prev_we    <= 1'b1;
          prev_waddr <= step_addr;
          prev_wdata <= v_reg;
          v_reg      <= comb_w;
          ph_reg     <= 2'd0;
          if(st_reg == STAGES-1) begin
            out_buf[s_reg] <= comb_w[SLICE_HI -: DOUT_W];
            if(s_reg == N_CH-1) begin
              state_reg       <= ST_OUT;
              oc_reg          <= {SCNT_W{1'b0}};
              out_pending_reg <= 1'b1;
            end
            else begin
              s_reg  <= s_reg + 1'b1;
              st_reg <= 4'd0;
            end
          end
          else begin
            st_reg <= st_reg + 1'b1;
          end
        end
        endcase
      end

      ST_OUT: begin
        if(~m_tvalid_reg | m_axis_tready) begin
          if(out_pending_reg) begin
            m_tdata_reg  <= out_buf[oc_reg];
            m_tvalid_reg <= 1'b1;
            if(oc_reg == N_CH-1)
              out_pending_reg <= 1'b0;
            else
              oc_reg <= oc_reg + 1'b1;
          end
          else begin
            m_tvalid_reg <= 1'b0;
            state_reg    <= ST_IDLE;
          end
        end
      end

      default: state_reg <= ST_IDLE;
      endcase
    end
  end

  xpm_memory_sdpram #(
    .MEMORY_SIZE(PREV_N*ACC_W),
    .MEMORY_PRIMITIVE("block"),
    .CLOCKING_MODE("common_clock"),
    .MEMORY_INIT_FILE("none"),
    .WRITE_DATA_WIDTH_A(ACC_W),
    .READ_DATA_WIDTH_B(ACC_W),
    .BYTE_WRITE_WIDTH_A(ACC_W),
    .ADDR_WIDTH_A(PADDR_W),
    .ADDR_WIDTH_B(PADDR_W),
    .READ_LATENCY_B(1),
    .WRITE_MODE_B("read_first"),
    .USE_MEM_INIT(0)
  ) prev_ram (
    .sleep(1'b0),
    .clka(aclk),
    .ena(1'b1),
    .wea(prev_we),
    .addra(prev_waddr),
    .dina(prev_wdata),
    .injectsbiterra(1'b0),
    .injectdbiterra(1'b0),
    .clkb(aclk),
    .enb(1'b1),
    .rstb(~aresetn),
    .regceb(1'b1),
    .addrb(step_addr),
    .doutb(prev_rdata),
    .sbiterrb(),
    .dbiterrb()
  );

endmodule
