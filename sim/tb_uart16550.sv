// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// tb_uart16550 -- self-checking bench for the 8250-family UART.
//
//   Five instances share one register bus, each with its own serial and
//   modem pins: VARIANT 0 (8250), 1 (16450), 2 (16550: FIFO reported, not
//   buffered), 3 (16550A), and a 16550A built with RESET_DIVISOR = 2 and a
//   non-default IRQ_VECTOR. The bench drives RXD itself to inject the parity,
//   framing and break errors loopback cannot produce, decodes TXD bit for bit,
//   and checks each result against 8250/16450/16550A data-sheet behavior:
//     * reset values of every register and pin on every variant; each
//       register's writable bits, DLAB banking, writes to LSR/MSR ignored,
//       chip-select gating;
//     * variant identity: SCR, IIR D7:6 after an FCR enable, whether the FIFO
//       buffers;
//     * on every variant in 16450 mode: receive, PE/FE/BI, sticky error bits,
//       overrun, a byte landing in the clock RBR is read, flags raised in the
//       clock LSR is read, the RDA/RLS/THRE interrupts, THRE rising with the
//       start bit and TEMT a frame later, a THR write while THRE is low
//       replacing the waiting byte, no character timeout;
//     * 16550A: the FCR FIFO clears (also while enabled, and in the clock the
//       shifter would take a byte) and the clear on an FCR0 change, all four
//       RX trigger levels, the character timeout at three frame sizes and its
//       clearing, the 16 transmit places filled from idle and behind a
//       shifting frame (the 17th write dropped, a write in the clock the
//       shifter takes the head taken),
//       RX overrun and a byte completing into a full FIFO with a read in that
//       clock, PE/FE/BI revealed as each character reaches the top, FIFOE over
//       the whole FIFO, every FIFO slot through every data and error bit;
//     * IIR priority across all five sources and each IER mask bit; the THRE
//       rules (cleared by an IIR read only when reported, re-armed by an
//       ETBEI write, set as THRE rises) and the FIFO-mode hold-back of the
//       THRE interrupt;
//     * TXD frames for five formats, break on TXD, loopback (TXD marking,
//       RTS#/DTR# inactive, the MSR mapping and deltas including TERI), the
//       modem-pin deltas, int_o gating by OUT2 and in loopback;
//     * RESET_DIVISOR and the baud tick periods; reset mid-activity.
//
//   CLK_HZ = 4 * REF_HZ, so divisor 1 is one bit every 64 clocks.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module tb_uart16550;

  localparam int unsigned ClkHz = 7_372_800;  // 4 * 1.8432 MHz
  localparam int unsigned RefHz = 1_843_200;
  localparam int RefClk = ClkHz / RefHz;  // clocks per reference period
  localparam int BitClk = 16 * RefClk;  // clocks per bit at divisor 1
  localparam int NDut = 5;
  localparam int DA = 3;  // the 16550A the FIFO tests run on
  localparam int DRst = 4;  // the 16550A built with RESET_DIVISOR = 2

  localparam logic [2:0] RegRbr = 3'd0, RegIer = 3'd1, RegIir = 3'd2, RegLcr = 3'd3;
  localparam logic [2:0] RegMcr = 3'd4, RegLsr = 3'd5, RegMsr = 3'd6, RegScr = 3'd7;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic                 nrst;
  logic [NDut-1:0]      cs;
  logic                 rds;
  logic                 wrs;
  logic [     2:0]      addr;
  logic [     7:0]      din;
  logic [NDut-1:0][7:0] dout;
  logic [NDut-1:0][7:0] vec;
  logic [NDut-1:0]      intr;
  logic [NDut-1:0]      txd;
  logic [NDut-1:0]      rxd;
  logic [NDut-1:0]      rxd_drv;
  logic [NDut-1:0]      ext_loop;
  logic [NDut-1:0]      cts_n;
  logic [NDut-1:0]      dsr_n;
  logic [NDut-1:0]      dcd_n;
  logic [NDut-1:0]      ri_n;
  logic [NDut-1:0]      rts_n;
  logic [NDut-1:0]      dtr_n;
  logic [NDut-1:0]      btick;
  logic [NDut-1:0]      stick;
  logic [NDut-1:0]      stb;
  logic [NDut-1:0]      txi;  // the transmitter's line, ahead of the loopback and break muxes
  logic [NDut-1:0]      txi_q;  // txi a clock ago
  logic [NDut-1:0]      thre;  // LSR THRE
  logic [NDut-1:0]      temt;  // LSR TEMT

  // RXD is the bench's driver, or the instance's own TXD (external loopback).
  assign rxd = (ext_loop & txd) | (~ext_loop & rxd_drv);

  for (genvar g = 0; g < NDut; g++) begin : g_dut
    uart16550 #(
        .VARIANT      ((g == DRst) ? 3 : g),
        .CLK_HZ       (ClkHz),
        .REF_HZ       (RefHz),
        .IRQ_VECTOR   ((g == DRst) ? 8'h3C : 8'hFF),
        .RESET_DIVISOR((g == DRst) ? 16'h0002 : 16'h0000)
    ) dut (
        .clk_i             (clk),
        .nrst_i            (nrst),
        .cs_i              (cs[g]),
        .addr_i            (addr),
        .rd_i              (rds),
        .wr_i              (wrs),
        .din_i             (din),
        .dout_o            (dout[g]),
        .int_o             (intr[g]),
        .int_vec_o         (vec[g]),
        .txd_o             (txd[g]),
        .rxd_i             (rxd[g]),
        .cts_n_i           (cts_n[g]),
        .dsr_n_i           (dsr_n[g]),
        .dcd_n_i           (dcd_n[g]),
        .ri_n_i            (ri_n[g]),
        .rts_n_o           (rts_n[g]),
        .dtr_n_o           (dtr_n[g]),
        .baud_bit_tick_o   (btick[g]),
        .baud_samp16_tick_o(stick[g])
    );
    // The receiver's strobe, so a read can be aimed at the clock a byte lands.
    assign stb[g]  = dut.rx_stb;
    // The transmit side, so THRE and the THRE interrupt can be timed against
    // the start bit and TEMT.
    assign txi[g]  = dut.txd_int;
    assign thre[g] = dut.lsr_val[5];
    assign temt[g] = dut.lsr_val[6];
  end

  int unsigned cyc = 0;  // free-running clock count
  // Bit ticks of the 16550A as its character timer counts them: a tick in the
  // clock of a receive strobe or an RBR read restarts the timer instead.
  int unsigned bt3 = 0;
  int unsigned mark3 = 0;  // bt3 at the 16550A's last receive strobe
  always_ff @(posedge clk) begin
    cyc   <= cyc + 1;
    txi_q <= txi;
    if (btick[DA] && !stb[DA]) bt3 <= bt3 + 1;
    if (stb[DA]) mark3 <= bt3;
  end

  int checks = 0;
  int errors = 0;
  int cur = 0;  // instance under test, for the failure message
  logic [8*24-1:0] sect = "";

  task automatic chk(input logic [8*64-1:0] name, input logic cond);
    checks++;
    if (cond !== 1'b1) begin
      errors++;
      $display("  FAIL [%0s, u%0d] %0s", sect, cur, name);
    end
  endtask

  task automatic chk8(input logic [8*64-1:0] name, input logic [7:0] got, input logic [7:0] want);
    checks++;
    if (got !== want) begin
      errors++;
      $display("  FAIL [%0s, u%0d] %0s: got %02h, want %02h", sect, cur, name, got, want);
    end
  endtask

  // Keep the observed value four-state at the task boundary. An unknown
  // measurement must fail even when zero is within the expected range.
  task automatic chk_rng(input logic [8*64-1:0] name, input logic signed [31:0] got, input int lo,
                         input int hi);
    checks++;
    if ((got >= lo && got <= hi) !== 1'b1) begin
      errors++;
      $display("  FAIL [%0s, u%0d] %0s: got %0d, want %0d..%0d", sect, cur, name, got, lo, hi);
    end
  endtask

  // ---- Clocking and the register bus (single-clock strobes) -----------------
  task automatic wait_clk(input int n);
    repeat (n) @(negedge clk);
  endtask

  task automatic wr_reg(input int s, input logic [2:0] a, input logic [7:0] d);
    @(negedge clk);
    cs    = '0;
    cs[s] = 1'b1;
    addr  = a;
    din   = d;
    wrs   = 1'b1;
    @(negedge clk);
    cs  = '0;
    wrs = 1'b0;
  endtask

  // A read strobed in the current clock (call at a negedge); d is the value
  // returned, sampled before the posedge that applies the read's side effects.
  task automatic rd_now(input int s, input logic [2:0] a, output logic [7:0] d);
    cs    = '0;
    cs[s] = 1'b1;
    addr  = a;
    rds   = 1'b1;
    #1 d = dout[s];
    @(negedge clk);
    cs  = '0;
    rds = 1'b0;
  endtask

  // A write strobed in the current clock (call at a negedge).
  task automatic wr_now(input int s, input logic [2:0] a, input logic [7:0] d);
    cs    = '0;
    cs[s] = 1'b1;
    addr  = a;
    din   = d;
    wrs   = 1'b1;
    @(negedge clk);
    cs  = '0;
    wrs = 1'b0;
  endtask

  task automatic rd_reg(input int s, input logic [2:0] a, output logic [7:0] d);
    @(negedge clk);
    rd_now(s, a, d);
  endtask

  task automatic expect_reg(input int s, input logic [2:0] a, input logic [7:0] want,
                            input logic [8*64-1:0] name);
    logic [7:0] v;
    rd_reg(s, a, v);
    chk8(name, v, want);
  endtask

  task automatic set_div(input int s, input logic [15:0] dv, input logic [7:0] lcr);
    wr_reg(s, RegLcr, 8'h80);
    wr_reg(s, RegRbr, dv[7:0]);
    wr_reg(s, RegIer, dv[15:8]);
    wr_reg(s, RegLcr, lcr);
  endtask

  task automatic pulse_reset();
    @(negedge clk);
    nrst = 1'b0;
    cs   = '0;
    rds  = 1'b0;
    wrs  = 1'b0;
    wait_clk(3);
    nrst = 1'b1;
    wait_clk(4);
  endtask

  task automatic reset_all();
    rxd_drv  = '1;
    ext_loop = '0;
    cts_n    = '1;
    dsr_n    = '1;
    dcd_n    = '1;
    ri_n     = '1;
    pulse_reset();
  endtask

  // Poll LSR until DR; lsr is the last value read.
  task automatic wait_dr(input int s, output logic [7:0] lsr);
    lsr = 8'h00;
    for (int g = 0; g < 20000 && !lsr[0]; g++) rd_reg(s, RegLsr, lsr);
  endtask

  task automatic wait_temt(input int s);
    logic [7:0] lsr;
    lsr = 8'h00;
    for (int g = 0; g < 40000 && !lsr[6]; g++) rd_reg(s, RegLsr, lsr);
  endtask

  // Wait until the instance's receiver strobes; returns at the negedge inside
  // the strobe clock, so an rd_now() here lands in that same clock.
  task automatic wait_stb(input int s);
    do @(negedge clk); while (stb[s] !== 1'b1);
  endtask

  // Wait for a bit tick; returns at the negedge before it, so a write issued
  // next lands just after it, a bit time ahead of the next.
  task automatic after_tick(input int s);
    do @(negedge clk); while (btick[s] !== 1'b1);
  endtask

  // Wait for THRE; t is the clock it is first seen, st whether a start bit
  // began in that clock.
  task automatic wait_thre(input int s, output int t, output bit st);
    int g;
    for (g = 0; g < 100000 && thre[s] !== 1'b1; g++) @(negedge clk);
    t  = int'(cyc);
    st = (txi[s] === 1'b0) && (txi_q[s] === 1'b1);
  endtask

  task automatic wait_int(input int s, output int t);
    int g;
    for (g = 0; g < 100000 && intr[s] !== 1'b1; g++) @(negedge clk);
    t = int'(cyc);
  endtask

  task automatic wait_temt_t(input int s, output int t);
    int g;
    for (g = 0; g < 100000 && temt[s] !== 1'b1; g++) @(negedge clk);
    t = int'(cyc);
  endtask

  // ---- Frame helpers ----------------------------------------------------------
  function automatic logic [7:0] dmask(input int n);
    return 8'hFF >> (8 - n);
  endfunction

  // Parity bit per LCR EPS/SP: even, odd, or stick (SP=1: EPS=0 mark, EPS=1 space).
  function automatic logic par_bit(input logic [7:0] d, input int n, input logic eps,
                                   input logic sp);
    logic x;
    x = ^(d & dmask(n));
    if (sp) return ~eps;
    return eps ? x : ~x;
  endfunction

  // Drive one frame on an instance's RXD at divisor 1: start, n data bits LSB
  // first, an optional parity bit, nstop stop bits at stopv, then marking. It
  // returns after the receiver has taken the byte (the strobe is mid-stop).
  task automatic rx_frame(input int s, input logic [7:0] d, input int n, input bit pen,
                          input logic par, input int nstop, input logic stopv);
    rxd_drv[s] = 1'b0;
    wait_clk(BitClk);
    for (int k = 0; k < n; k++) begin
      rxd_drv[s] = d[k];
      wait_clk(BitClk);
    end
    if (pen) begin
      rxd_drv[s] = par;
      wait_clk(BitClk);
    end
    rxd_drv[s] = stopv;
    wait_clk(BitClk * nstop);
    rxd_drv[s] = 1'b1;
    if (!stopv) wait_clk(BitClk);  // marking again before the next start bit
  endtask

  // A frame in the format LCR selects, optionally with bad parity or a stop bit
  // sampled low (a framing error).
  task automatic rx_lcr(input int s, input logic [7:0] lcr, input logic [7:0] d, input bit bad_par,
                        input bit bad_stop);
    int n;
    n = 5 + lcr[1:0];
    rx_frame(s, d, n, lcr[3], par_bit(d, n, lcr[4], lcr[5]) ^ bad_par, lcr[2] ? 2 : 1, !bad_stop);
  endtask

  task automatic rx8n1(input int s, input logic [7:0] d);
    rx_frame(s, d, 8, 1'b0, 1'b0, 1, 1'b1);
  endtask

  // Hold RXD spacing for nbits bit times (a break), then marking for two.
  task automatic rx_break(input int s, input int nbits);
    rxd_drv[s] = 1'b0;
    wait_clk(BitClk * nbits);
    rxd_drv[s] = 1'b1;
    wait_clk(BitClk * 2);
  endtask

  // Capture one frame from TXD: wait for the start bit, then sample each of
  // nbits bits (start included) at mid-bit. ok = 0 if no start bit came.
  task automatic tx_capture(input int s, input int nbits, input int bclk, output logic [15:0] bits,
                            output bit ok);
    int g;
    bits = 16'h0000;
    for (g = 0; g < 100000 && txd[s] !== 1'b0; g++) @(negedge clk);
    ok = (txd[s] === 1'b0);
    wait_clk(bclk / 2);
    for (int k = 0; k < nbits; k++) begin
      bits[k] = txd[s];
      wait_clk(bclk);
    end
  endtask

  // Clocks from one baud tick to the next (samp = 1: the 16x tick).
  task automatic tick_period(input int s, input bit samp, output int p);
    int g;
    p = 0;
    for (g = 0; g < 100000 && ((samp ? stick[s] : btick[s]) !== 1'b1); g++) @(negedge clk);
    @(negedge clk);
    p = 1;
    for (g = 0; g < 100000 && ((samp ? stick[s] : btick[s]) !== 1'b1); g++) begin
      @(negedge clk);
      p++;
    end
  endtask

  // Wait for the 16550A's IIR to show want; returns the bit ticks since its
  // last receive strobe, or since `from` when from_rd (an RBR read restarts
  // the character timer).
  task automatic wait_iir(input logic [7:0] want, input int max_ticks, input bit from_rd,
                          input int unsigned from, output int elapsed);
    logic [7:0] v;
    int unsigned start;
    start = bt3;
    v     = 8'h00;
    while (v !== want && (bt3 - start) < max_ticks) rd_reg(DA, RegIir, v);
    elapsed = from_rd ? int'(bt3 - from) : int'(bt3 - mark3);
    if (v !== want) elapsed = -1;
  endtask

  // ==========================================================================
  //  Sections
  // ==========================================================================

  task automatic t_reset_values();
    sect = "reset values";
    reset_all();
    for (int s = 0; s < NDut; s++) begin
      cur = s;
      expect_reg(s, RegLsr, 8'h60, "LSR = THRE | TEMT");
      expect_reg(s, RegIir, 8'h01, "IIR = no interrupt");
      expect_reg(s, RegIer, 8'h00, "IER = 0");
      expect_reg(s, RegLcr, 8'h00, "LCR = 0");
      expect_reg(s, RegMcr, 8'h00, "MCR = 0");
      expect_reg(s, RegMsr, 8'h00, "MSR = 0 (pins inactive)");
      expect_reg(s, RegScr, 8'h00, "SCR = 0");
      wr_reg(s, RegLcr, 8'h80);
      expect_reg(s, RegRbr, (s == DRst) ? 8'h02 : 8'h00, "DLL = RESET_DIVISOR[7:0]");
      expect_reg(s, RegIer, 8'h00, "DLM = RESET_DIVISOR[15:8]");
      wr_reg(s, RegLcr, 8'h00);
      chk("int_o low", intr[s] === 1'b0);
      chk("TXD marking", txd[s] === 1'b1);
      chk("RTS# and DTR# inactive", rts_n[s] === 1'b1 && dtr_n[s] === 1'b1);
      chk8("int_vec_o = IRQ_VECTOR", vec[s], (s == DRst) ? 8'h3C : 8'hFF);
    end
  endtask

  task automatic t_registers(input int s);
    logic [7:0] scr_want;
    cur  = s;
    sect = "registers";
    reset_all();
    // IER: four bits.
    wr_reg(s, RegIer, 8'hFF);
    expect_reg(s, RegIer, 8'h0F, "IER keeps D3:0");
    wr_reg(s, RegIer, 8'hA5);
    expect_reg(s, RegIer, 8'h05, "IER = A5 & 0F");
    // LCR: every bit; D6 is break, so TXD goes spacing while it is set.
    wr_reg(s, RegLcr, 8'h7F);
    expect_reg(s, RegLcr, 8'h7F, "LCR = 7F");
    chk("LCR D6 drives TXD spacing", txd[s] === 1'b0);
    wr_reg(s, RegLcr, 8'h00);
    expect_reg(s, RegLcr, 8'h00, "LCR = 00");
    chk("TXD marking again", txd[s] === 1'b1);
    // DLAB = 1: offsets 0 and 1 are DLL and DLM, every bit.
    wr_reg(s, RegLcr, 8'hFF);
    expect_reg(s, RegLcr, 8'hFF, "LCR = FF");
    wr_reg(s, RegLcr, 8'h80);
    wr_reg(s, RegRbr, 8'hFF);
    wr_reg(s, RegIer, 8'hFF);
    expect_reg(s, RegRbr, 8'hFF, "DLL = FF");
    expect_reg(s, RegIer, 8'hFF, "DLM = FF (not IER)");
    wr_reg(s, RegRbr, 8'hA5);
    wr_reg(s, RegIer, 8'h5A);
    expect_reg(s, RegRbr, 8'hA5, "DLL = A5");
    expect_reg(s, RegIer, 8'h5A, "DLM = 5A");
    wr_reg(s, RegRbr, 8'h00);
    wr_reg(s, RegIer, 8'h00);
    expect_reg(s, RegRbr, 8'h00, "DLL = 00");
    expect_reg(s, RegIer, 8'h00, "DLM = 00");
    expect_reg(s, RegLsr, 8'h60, "a DLL write does not load the THR");
    wr_reg(s, RegLcr, 8'h00);
    expect_reg(s, RegIer, 8'h05, "IER untouched by the DLM writes");
    wr_reg(s, RegIer, 8'h00);
    expect_reg(s, RegIer, 8'h00, "IER = 0");
    // MCR: five bits; RTS# and DTR# follow D1 and D0 outside loopback.
    wr_reg(s, RegMcr, 8'hFF);
    expect_reg(s, RegMcr, 8'h1F, "MCR keeps D4:0");
    chk("loopback holds RTS# and DTR# inactive", rts_n[s] === 1'b1 && dtr_n[s] === 1'b1);
    expect_reg(s, RegMsr, 8'hFB, "loopback with all four outputs: levels, DCTS DDSR DDCD");
    wr_reg(s, RegMcr, 8'h01);
    expect_reg(s, RegMsr, 8'h0F, "leaving loopback, pins inactive: all four deltas");
    chk("MCR DTR -> DTR# low, RTS# high", dtr_n[s] === 1'b0 && rts_n[s] === 1'b1);
    wr_reg(s, RegMcr, 8'h02);
    chk("MCR RTS -> RTS# low, DTR# high", rts_n[s] === 1'b0 && dtr_n[s] === 1'b1);
    wr_reg(s, RegMcr, 8'h0C);
    expect_reg(s, RegMcr, 8'h0C, "MCR = OUT1 | OUT2");
    chk("OUT1/OUT2 leave RTS#/DTR# inactive", rts_n[s] === 1'b1 && dtr_n[s] === 1'b1);
    wr_reg(s, RegMcr, 8'h00);
    expect_reg(s, RegMsr, 8'h00, "MSR: no deltas from MCR writes outside loopback");
    // SCR: absent on the 8250.
    scr_want = (s == 0) ? 8'h00 : 8'hFF;
    wr_reg(s, RegScr, 8'hFF);
    expect_reg(s, RegScr, scr_want, "SCR = FF (8250: none)");
    wr_reg(s, RegScr, 8'h00);
    expect_reg(s, RegScr, 8'h00, "SCR = 00");
    wr_reg(s, RegScr, 8'h5A);
    expect_reg(s, RegScr, (s == 0) ? 8'h00 : 8'h5A, "SCR = 5A (8250: none)");
    // LSR and MSR ignore writes.
    wr_reg(s, RegLsr, 8'h9F);
    expect_reg(s, RegLsr, 8'h60, "LSR write ignored");
    wr_reg(s, RegMsr, 8'h0F);
    expect_reg(s, RegMsr, 8'h00, "MSR write ignored");
  endtask

  // The register bus is shared: an access reaches only the selected instance.
  task automatic t_chip_select();
    sect = "chip select";
    cur  = DA;
    wr_reg(1, RegScr, 8'h3C);
    wr_reg(2, RegScr, 8'h3C);
    wr_reg(DA, RegScr, 8'hC3);
    expect_reg(1, RegScr, 8'h3C, "16450 SCR unchanged by a write to another");
    expect_reg(2, RegScr, 8'h3C, "16550 SCR unchanged by a write to another");
    expect_reg(DA, RegScr, 8'hC3, "16550A SCR written");
  endtask

  task automatic t_variants();
    sect = "variants";
    reset_all();
    for (int s = 0; s < 4; s++) begin
      cur = s;
      wr_reg(s, RegIir, 8'h01);
      expect_reg(s, RegIir, (s == 3) ? 8'hC1 : (s == 2) ? 8'h81 : 8'h01,
                 "IIR D7:6 after FCR0 = 1: 00, 00, 10, 11");
      wr_reg(s, RegIer, 8'h02);
      expect_reg(s, RegIir, (s == 3) ? 8'hC2 : (s == 2) ? 8'h82 : 8'h02,
                 "IIR THRE carries the FIFO status bits");
      wr_reg(s, RegIer, 8'h00);
      wr_reg(s, RegIir, 8'h00);
      expect_reg(s, RegIir, 8'h01, "IIR D7:6 = 00 after FCR0 = 0");
      // Whether the FIFO buffers: two bytes unread overrun on all but the 16550A.
      set_div(s, 16'd1, 8'h03);
      wr_reg(s, RegIir, 8'h07);
      rx8n1(s, 8'h31);
      rx8n1(s, 8'h32);
      expect_reg(s, RegLsr, (s == 3) ? 8'h61 : 8'h63, "two bytes: OE unless the FIFO buffers");
      expect_reg(s, RegRbr, (s == 3) ? 8'h31 : 8'h32, "first read");
      expect_reg(s, RegLsr, (s == 3) ? 8'h61 : 8'h60, "second byte still held only by the 16550A");
    end
  endtask

  function automatic logic [7:0] fmt_lcr(input int i);
    case (i)
      0: return 8'h00;  // 5N1
      1: return 8'h01;  // 6N1
      2: return 8'h02;  // 7N1
      3: return 8'h03;  // 8N1
      4: return 8'h04;  // 5N1.5
      5: return 8'h07;  // 8N2
      6: return 8'h0B;  // 8O1
      7: return 8'h1A;  // 7E1
      8: return 8'h1F;  // 8E2
      9: return 8'h2B;  // 8 mark 1
      10: return 8'h3B;  // 8 space 1
      default: return 8'h28;  // 5 mark 1
    endcase
  endfunction

  // One instance in 16450 mode (the 16550A with its FIFO off).
  task automatic t_16450_mode(input int s);
    logic [7:0] v, f, lcr, d;
    int unsigned t0;
    int t1, t2;
    bit st;
    cur  = s;
    sect = "16450 mode";
    reset_all();
    set_div(s, 16'd1, 8'h03);
    f = (s == 2) ? 8'h80 : 8'h00;  // IIR D7:6 with FCR0 set (16550 only)
    // An FCR write with FCR0 = 0 programs nothing on the 16550A.
    wr_reg(s, RegIir, (s == DA) ? 8'hC6 : 8'hC7);
    expect_reg(s, RegIir, f | 8'h01, "IIR: no interrupt");

    // ---- Loopback round trip; on the 16550A every frame format -------------
    wr_reg(s, RegMcr, 8'h10);
    for (int i = 0; i < ((s == DA) ? 12 : 1); i++) begin
      lcr = (s == DA) ? fmt_lcr(i) : 8'h03;
      wr_reg(s, RegLcr, lcr);
      for (int j = 0; j < 4; j++) begin
        d = (j == 0) ? 8'hFF : (j == 1) ? 8'h00 : (j == 2) ? 8'hA5 : 8'h5A;
        wr_reg(s, RegRbr, d);
        wait_dr(s, v);
        chk8("loopback: no error bits", v & 8'h9E, 8'h00);
        expect_reg(s, RegRbr, d & dmask(5 + lcr[1:0]), "loopback RBR");
      end
      wait_temt(s);
    end
    wr_reg(s, RegLcr, 8'h03);
    wr_reg(s, RegMcr, 8'h00);
    expect_reg(s, RegMsr, 8'h00, "MSR clear");

    // ---- Receive from the pin ------------------------------------------------
    rx8n1(s, 8'hC3);
    expect_reg(s, RegLsr, 8'h61, "DR");
    expect_reg(s, RegIir, f | 8'h01, "IER = 0: no interrupt");
    expect_reg(s, RegRbr, 8'hC3, "RBR");
    expect_reg(s, RegLsr, 8'h60, "DR cleared by the read");

    // RDA interrupt, gated to int_o by OUT2; no character timeout in this mode.
    wr_reg(s, RegIer, 8'h01);
    rx8n1(s, 8'h3C);
    expect_reg(s, RegIir, f | 8'h04, "IIR RDA");
    chk("int_o low with OUT2 clear", intr[s] === 1'b0);
    wr_reg(s, RegMcr, 8'h08);
    chk("int_o high with OUT2 set", intr[s] === 1'b1);
    wait_clk(50 * BitClk);
    expect_reg(s, RegIir, f | 8'h04, "still RDA after 5 character times: no timeout");
    expect_reg(s, RegRbr, 8'h3C, "RBR");
    expect_reg(s, RegIir, f | 8'h01, "RDA cleared by the read");
    chk("int_o low", intr[s] === 1'b0);

    // ---- Line status: PE, FE, BI; RLS outranks RDA ---------------------------
    wr_reg(s, RegIer, 8'h05);
    wr_reg(s, RegLcr, 8'h1B);  // 8E1
    rx_lcr(s, 8'h1B, 8'h81, 1'b1, 1'b0);
    expect_reg(s, RegIir, f | 8'h06, "IIR RLS on a parity error");
    expect_reg(s, RegLsr, 8'h65, "LSR PE (no FIFOE in 16450 mode)");
    expect_reg(s, RegIir, f | 8'h04, "LSR read clears RLS, RDA remains");
    expect_reg(s, RegRbr, 8'h81, "the errored byte is delivered");
    expect_reg(s, RegLsr, 8'h60, "clean");
    wr_reg(s, RegLcr, 8'h03);
    rx_lcr(s, 8'h03, 8'h55, 1'b0, 1'b1);
    expect_reg(s, RegLsr, 8'h69, "LSR FE");
    expect_reg(s, RegRbr, 8'h55, "RBR after FE");
    rx_break(s, 12);
    expect_reg(s, RegLsr, 8'h79, "LSR BI | FE");
    expect_reg(s, RegRbr, 8'h00, "a break reads as 00");
    wr_reg(s, RegLcr, 8'h0B);  // 8O1: the all-zero parity slot is also a PE
    rx_break(s, 12);
    expect_reg(s, RegLsr, 8'h7D, "LSR BI | FE | PE under odd parity");
    expect_reg(s, RegRbr, 8'h00, "break RBR");
    wr_reg(s, RegLcr, 8'h03);
    expect_reg(s, RegLsr, 8'h60, "clean");

    // Error bits are sticky until an LSR read, not cleared by a clean byte.
    rx_lcr(s, 8'h03, 8'h11, 1'b0, 1'b1);
    expect_reg(s, RegRbr, 8'h11, "FE byte read, LSR not read");
    rx8n1(s, 8'h22);
    expect_reg(s, RegLsr, 8'h69, "FE survives the next clean byte");
    expect_reg(s, RegLsr, 8'h61, "cleared by the LSR read");
    expect_reg(s, RegRbr, 8'h22, "RBR");

    // ---- Overrun ---------------------------------------------------------------
    rx8n1(s, 8'hA1);
    rx8n1(s, 8'hA2);
    expect_reg(s, RegIir, f | 8'h06, "IIR RLS on overrun");
    expect_reg(s, RegLsr, 8'h63, "LSR OE | DR");
    expect_reg(s, RegLsr, 8'h61, "OE cleared by the LSR read");
    expect_reg(s, RegRbr, 8'hA2, "the newer byte overwrote the RBR");
    expect_reg(s, RegLsr, 8'h60, "empty");

    // A byte landing in the clock the RBR is read replaces it, without OE.
    rx8n1(s, 8'hB1);
    fork
      rx8n1(s, 8'hB2);
      begin
        wait_stb(s);
        rd_now(s, RegRbr, v);
      end
    join
    chk8("read in the landing clock returns the old byte", v, 8'hB1);
    expect_reg(s, RegLsr, 8'h61, "no OE, the new byte is held");
    expect_reg(s, RegRbr, 8'hB2, "the new byte");

    // Flags raised in the clock LSR is read survive that read.
    rx8n1(s, 8'hC1);
    fork
      rx_lcr(s, 8'h03, 8'hC2, 1'b0, 1'b1);
      begin
        wait_stb(s);
        rd_now(s, RegLsr, v);
      end
    join
    chk8("LSR read in the landing clock shows the old state", v, 8'h61);
    expect_reg(s, RegLsr, 8'h6B, "OE and FE raised in that clock survive it");
    expect_reg(s, RegRbr, 8'hC2, "RBR");

    // ---- THRE, TEMT and the THRE interrupt -------------------------------------
    wr_reg(s, RegIer, 8'h02);
    expect_reg(s, RegIir, f | 8'h02, "ETBEI with THR empty: THRE");
    expect_reg(s, RegIir, f | 8'h01, "the IIR read that reported THRE cleared it");
    wr_reg(s, RegMcr, 8'h18);
    // The byte stays in the THR until it enters the shifter at its start bit,
    // on the bit tick after the write.
    after_tick(s);
    wr_reg(s, RegRbr, 8'h96);
    t0 = cyc;
    rd_reg(s, RegLsr, v);
    chk8("THR written: neither THRE nor TEMT before the start bit", v & 8'h60, 8'h00);
    expect_reg(s, RegIir, f | 8'h01, "no THRE interrupt before the start bit");
    wait_thre(s, t1, st);
    chk("THRE rises in the clock the start bit begins", st);
    expect_reg(s, RegIir, f | 8'h02, "the THRE interrupt rises with it");
    rd_reg(s, RegLsr, v);
    chk8("the frame shifting: THRE, not TEMT", v & 8'h60, 8'h20);
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'h96, "loopback byte");
    wait_temt(s);
    chk_rng("TEMT after one 10-bit frame", int'(cyc - t0), 10 * BitClk, 11 * BitClk + 16);
    // A byte written behind a shifting frame waits in the THR and follows it
    // back to back; THRE rises as it starts.
    after_tick(s);
    wr_reg(s, RegRbr, 8'h97);
    wait_thre(s, t1, st);
    wait_clk(BitClk);  // the first byte is shifting
    wr_reg(s, RegRbr, 8'h98);
    rd_reg(s, RegLsr, v);
    chk8("a byte behind the shifter: THRE low", v & 8'h60, 8'h00);
    wait_thre(s, t2, st);
    chk("THRE rises as the waiting byte starts", st);
    chk_rng("one frame after the first started", t2 - t1, 10 * BitClk, 10 * BitClk);
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'h97, "first");
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'h98, "second");
    wait_temt(s);
    // With the baud stopped the byte waits in the THR and THRE stays low; a
    // THR write while THRE is low replaces the waiting byte.
    set_div(s, 16'd0, 8'h03);
    wr_reg(s, RegRbr, 8'hAA);
    expect_reg(s, RegLsr, 8'h00, "the byte waits: neither THRE nor TEMT");
    wr_reg(s, RegRbr, 8'hBB);
    expect_reg(s, RegLsr, 8'h00, "still one byte waiting");
    expect_reg(s, RegIir, f | 8'h01, "a THR write clears THRE");
    set_div(s, 16'd1, 8'h03);
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'hBB, "the write made while THRE was low replaced the waiting byte");
    wait_temt(s);
    wait_clk(12 * BitClk);
    expect_reg(s, RegLsr, 8'h60, "one byte sent");
    // A write in the clock the shifter takes the waiting byte waits behind it.
    after_tick(s);
    wr_reg(s, RegRbr, 8'hC5);
    after_tick(s);
    wr_now(s, RegRbr, 8'hC6);
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'hC5, "the byte the shifter took in that clock");
    wait_dr(s, v);
    expect_reg(s, RegRbr, 8'hC6, "the byte written in it, next");
    wait_temt(s);
  endtask

  // TXD on the wire, bit for bit, in five formats.
  task automatic t_tx_wire();
    logic [15:0] bits, want;
    logic [7:0] lcr, d;
    bit ok;
    int n, nb;
    cur  = DA;
    sect = "TXD frames";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    for (int i = 0; i < 5; i++) begin
      case (i)
        0: begin  // 8N1
          lcr = 8'h03;
          d   = 8'hA5;
        end
        1: begin  // 6O2
          lcr = 8'h0D;
          d   = 8'h2B;
        end
        2: begin  // 5 mark
          lcr = 8'h28;
          d   = 8'h15;
        end
        3: begin  // 5 space
          lcr = 8'h38;
          d   = 8'h1F;
        end
        default: begin  // 8E2
          lcr = 8'h1F;
          d   = 8'h96;
        end
      endcase
      n    = 5 + lcr[1:0];
      want = 16'h0000;
      nb   = 1;
      for (int k = 0; k < n; k++) want[nb++] = d[k];
      if (lcr[3]) want[nb++] = par_bit(d, n, lcr[4], lcr[5]);
      want[nb++] = 1'b1;
      if (lcr[2]) want[nb++] = 1'b1;
      wr_reg(DA, RegLcr, lcr);
      wr_reg(DA, RegRbr, d);
      tx_capture(DA, nb, BitClk, bits, ok);
      chk("TXD start bit", ok);
      chk8("TXD frame low bits", bits[7:0], want[7:0]);
      chk8("TXD frame high bits", bits[15:8], want[15:8]);
      wait_temt(DA);
      chk("TXD marking after the frame", txd[DA] === 1'b1);
    end
  endtask

  task automatic t_break();
    logic [7:0] v;
    bit low;
    cur  = DA;
    sect = "break";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    ext_loop[DA] = 1'b1;  // the receiver hears TXD
    wr_reg(DA, RegLcr, 8'h43);
    chk("break drives TXD spacing", txd[DA] === 1'b0);
    wait_clk(14 * BitClk);
    expect_reg(DA, RegLsr, 8'h79, "the receiver sees the break: BI | FE");
    expect_reg(DA, RegRbr, 8'h00, "break byte");
    // The transmitter keeps shifting under break; only TXD is forced.
    wr_reg(DA, RegRbr, 8'h55);
    low = 1'b1;
    for (int k = 0; k < 12 * 16; k++) begin
      wait_clk(BitClk / 16);
      if (txd[DA] !== 1'b0) low = 1'b0;
    end
    chk("TXD stays spacing through a frame sent under break", low);
    expect_reg(DA, RegLsr, 8'h60, "that frame shifted out: TEMT");
    wr_reg(DA, RegLcr, 8'h03);
    chk("TXD marking after break", txd[DA] === 1'b1);
    wait_clk(2 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "nothing received after the break ends");
    // In loopback the break reaches the receiver but TXD stays marking.
    ext_loop[DA] = 1'b0;
    wr_reg(DA, RegMcr, 8'h10);
    wr_reg(DA, RegLcr, 8'h43);
    low = 1'b0;
    for (int k = 0; k < 14 * 16; k++) begin
      wait_clk(BitClk / 16);
      if (txd[DA] !== 1'b1) low = 1'b1;
    end
    chk("loopback holds TXD marking under break", !low);
    expect_reg(DA, RegLsr, 8'h79, "loopback break: BI | FE");
    wr_reg(DA, RegLcr, 8'h03);
    expect_reg(DA, RegRbr, 8'h00, "break byte");
  endtask

  task automatic t_fifo_clears();
    logic [7:0] v;
    cur  = DA;
    sect = "FIFO clears";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'h01);
    expect_reg(DA, RegIir, 8'hC1, "FIFO on");
    rx8n1(DA, 8'h11);
    rx8n1(DA, 8'h22);
    rx8n1(DA, 8'h33);
    expect_reg(DA, RegLsr, 8'h61, "three bytes held");
    wr_reg(DA, RegIir, 8'h03);  // FCR1 while enabled
    expect_reg(DA, RegLsr, 8'h60, "RX FIFO cleared while enabled");
    rx8n1(DA, 8'h44);
    rx8n1(DA, 8'h55);
    expect_reg(DA, RegRbr, 8'h44, "first byte after the clear");
    expect_reg(DA, RegRbr, 8'h55, "second byte after the clear");
    expect_reg(DA, RegLsr, 8'h60, "empty");
    // FCR0 1 -> 0 and 0 -> 1 both clear.
    rx8n1(DA, 8'h66);
    wr_reg(DA, RegIir, 8'h00);
    expect_reg(DA, RegLsr, 8'h60, "leaving FIFO mode clears the RX FIFO");
    expect_reg(DA, RegIir, 8'h01, "16450 mode");
    rx8n1(DA, 8'h77);
    expect_reg(DA, RegLsr, 8'h61, "byte in the 16450-mode RBR");
    wr_reg(DA, RegIir, 8'h01);
    expect_reg(DA, RegLsr, 8'h60, "entering FIFO mode clears the RBR");
    wr_reg(DA, RegIir, 8'h01);
    rx8n1(DA, 8'h78);
    wr_reg(DA, RegIir, 8'h01);  // FCR0 already 1: no clear
    expect_reg(DA, RegRbr, 8'h78, "FCR0 rewritten as 1 keeps the FIFO");
    // TX FIFO clear: only the byte in the shifter goes out.
    ext_loop[DA] = 1'b1;
    wr_reg(DA, RegIir, 8'h07);
    wr_reg(DA, RegRbr, 8'h81);
    wait_clk(BitClk + 16);
    for (int k = 2; k <= 10; k++) wr_reg(DA, RegRbr, 8'h80 + 8'(k));
    expect_reg(DA, RegLsr, 8'h00, "TX FIFO loaded: neither THRE nor TEMT");
    wr_reg(DA, RegIer, 8'h02);
    expect_reg(DA, RegIir, 8'hC1, "no THRE while the FIFO holds bytes");
    wr_reg(DA, RegIir, 8'h05);  // FCR2
    expect_reg(DA, RegLsr, 8'h20, "TX FIFO cleared: THRE, the shifter busy");
    expect_reg(DA, RegIir, 8'hC2, "the clear raises the THRE interrupt");
    wr_reg(DA, RegIer, 8'h00);
    wait_temt(DA);
    wait_clk(BitClk);
    expect_reg(DA, RegRbr, 8'h81, "the byte in the shifter");
    expect_reg(DA, RegLsr, 8'h60, "nothing from the cleared FIFO");
    // A clear in the clock the shifter would take the waiting byte discards
    // it: FCR2 in FIFO mode, then an FCR0 change in 16450 mode.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h91);
    after_tick(DA);
    wr_now(DA, RegIir, 8'h05);
    wait_clk(12 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "FCR2 in the load clock: nothing sent");
    wr_reg(DA, RegIir, 8'h00);  // 16450 mode
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h92);
    after_tick(DA);
    wr_now(DA, RegIir, 8'h01);
    wait_clk(12 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "FCR0 change in the load clock: nothing sent");
    // Away from the load clock, an FCR0 change discards the waiting THR byte.
    wr_reg(DA, RegIir, 8'h00);
    set_div(DA, 16'd0, 8'h03);
    wr_reg(DA, RegRbr, 8'h93);
    expect_reg(DA, RegLsr, 8'h00, "a byte waits in the THR");
    wr_reg(DA, RegIir, 8'h01);
    expect_reg(DA, RegLsr, 8'h60, "the FCR0 change discards it: THRE and TEMT");
    set_div(DA, 16'd1, 8'h03);
    wait_clk(12 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "nothing sent");
  endtask

  task automatic t_triggers();
    logic [7:0] v;
    int lvl;
    cur  = DA;
    sect = "RX trigger levels";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegMcr, 8'h08);
    wr_reg(DA, RegIer, 8'h01);
    for (int t = 0; t < 4; t++) begin
      lvl = (t == 0) ? 1 : (t == 1) ? 4 : (t == 2) ? 8 : 14;
      wr_reg(DA, RegIir, 8'(t << 6) | 8'h07);
      for (int k = 0; k < lvl - 1; k++) rx8n1(DA, 8'(16 * t + k));
      expect_reg(DA, RegIir, 8'hC1, "one below the trigger: no RDA");
      chk("int_o low below the trigger", intr[DA] === 1'b0);
      expect_reg(DA, RegLsr, (lvl > 1) ? 8'h61 : 8'h60, "DR with any byte held");
      rx8n1(DA, 8'(16 * t + lvl - 1));
      expect_reg(DA, RegIir, 8'hC4, "at the trigger: RDA");
      chk("int_o high at the trigger", intr[DA] === 1'b1);
      for (int k = 0; k < lvl; k++) begin
        expect_reg(DA, RegRbr, 8'(16 * t + k), "FIFO order");
        if (k == 0) expect_reg(DA, RegIir, 8'hC1, "one read drops below the trigger");
      end
      expect_reg(DA, RegLsr, 8'h60, "drained");
    end
  endtask

  task automatic t_timeout();
    int e;
    int unsigned t0;
    logic [7:0] v;
    cur  = DA;
    sect = "character timeout";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'hC7);  // trigger 14
    wr_reg(DA, RegIer, 8'h01);
    wr_reg(DA, RegMcr, 8'h08);
    rx8n1(DA, 8'hE1);
    rx8n1(DA, 8'hE2);
    rx8n1(DA, 8'hE3);
    expect_reg(DA, RegIir, 8'hC1, "below the trigger, timer running");
    wait_iir(8'hCC, 60, 1'b0, 0, e);
    chk_rng("8N1: timeout on the first tick past 4 x 10 bits", e, 41, 41);
    chk("int_o high on timeout", intr[DA] === 1'b1);
    rd_reg(DA, RegRbr, v);
    t0 = bt3;
    chk8("RBR", v, 8'hE1);
    expect_reg(DA, RegIir, 8'hC1, "an RBR read clears the timeout");
    wait_iir(8'hCC, 60, 1'b1, t0, e);
    chk_rng("the read restarted the timer", e, 41, 41);
    expect_reg(DA, RegRbr, 8'hE2, "RBR");
    wait_clk(30 * BitClk);
    rx8n1(DA, 8'hE4);
    wait_iir(8'hCC, 60, 1'b0, 0, e);
    chk_rng("a received byte restarts the timer", e, 41, 41);
    expect_reg(DA, RegRbr, 8'hE3, "RBR");
    expect_reg(DA, RegRbr, 8'hE4, "RBR");
    wait_clk(50 * BitClk);
    expect_reg(DA, RegIir, 8'hC1, "no timeout with the FIFO empty");
    // The four character times follow the frame size.
    wr_reg(DA, RegLcr, 8'h00);  // 5N1: 7 bits
    rx_lcr(DA, 8'h00, 8'h15, 1'b0, 1'b0);
    wait_iir(8'hCC, 60, 1'b0, 0, e);
    chk_rng("5N1: the first tick past 4 x 7 bits", e, 29, 29);
    expect_reg(DA, RegRbr, 8'h15, "RBR");
    wr_reg(DA, RegLcr, 8'h1F);  // 8E2: 12 bits
    rx_lcr(DA, 8'h1F, 8'h6E, 1'b0, 1'b0);
    wait_iir(8'hCC, 60, 1'b0, 0, e);
    chk_rng("8E2: the first tick past 4 x 12 bits", e, 49, 49);
    expect_reg(DA, RegRbr, 8'h6E, "RBR");
    expect_reg(DA, RegIir, 8'hC1, "clear");
  endtask

  // Loopback receive until TEMT with the RX FIFO empty, into rxbuf / rxn; the
  // clocks LSR first showed THRE and TEMT.
  logic [7:0] rxbuf[24];
  int rxn;
  task automatic drain_rx(output int t_thre, output int t_temt);
    logic [7:0] v, b;
    rxn    = 0;
    t_thre = -1;
    t_temt = -1;
    v      = 8'h00;
    for (int g = 0; g < 40000 && (t_temt < 0 || v[0]); g++) begin
      rd_reg(DA, RegLsr, v);
      if (t_thre < 0 && v[5]) begin
        t_thre = int'(cyc);
        chk("THRE before TEMT", !v[6]);
      end
      if (t_temt < 0 && v[6]) t_temt = int'(cyc);
      if (v[0]) begin
        rd_reg(DA, RegRbr, b);
        rxbuf[rxn] = b;
        if (rxn < 23) rxn++;
      end
    end
  endtask

  // The TX FIFO filled to full and drained through loopback, with THRE/TEMT:
  // 16 places behind the shifter.
  task automatic t_tx_fill();
    int t_thre, t_temt;
    bit st;
    cur  = DA;
    sect = "TX FIFO fill/drain";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'h07);
    wr_reg(DA, RegMcr, 8'h10);
    // Behind a shifting frame: the 16th write is taken, the 17th dropped.
    wr_reg(DA, RegRbr, 8'h01);  // into the shifter
    wait_clk(BitClk + 16);
    wr_reg(DA, RegRbr, 8'h02);
    expect_reg(DA, RegLsr, 8'h00, "a byte behind the shifter: neither THRE nor TEMT");
    for (int k = 3; k <= 17; k++) wr_reg(DA, RegRbr, 8'(k * 13));  // the 16th is 17 * 13
    expect_reg(DA, RegLsr, 8'h00, "FIFO full: neither THRE nor TEMT");
    wr_reg(DA, RegRbr, 8'hEE);  // the 17th, dropped
    drain_rx(t_thre, t_temt);
    wait_clk(12 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "nothing after the 17th byte");
    chk_rng("bytes through behind a shifting frame: shifter + 16", rxn, 17, 17);
    chk8("the shifter's byte", rxbuf[0], 8'h01);
    chk8("the first of the 16", rxbuf[1], 8'h02);
    for (int k = 2; k < 17; k++) chk8("FIFO byte order", rxbuf[k], 8'((k + 1) * 13));
    // THRE rises as the last byte enters the shifter: one frame remains.
    chk_rng("TEMT one frame after THRE", t_temt - t_thre, 10 * BitClk - 16, 10 * BitClk + 16);
    // From idle: a burst between two bit ticks fills the 16 places before the
    // first byte starts, and the 17th is dropped.
    after_tick(DA);
    for (int k = 0; k < 17; k++) wr_reg(DA, RegRbr, 8'h60 + 8'(k));
    expect_reg(DA, RegLsr, 8'h00, "burst from idle: neither THRE nor TEMT");
    drain_rx(t_thre, t_temt);
    wait_clk(12 * BitClk);
    expect_reg(DA, RegLsr, 8'h60, "nothing after the 16th byte");
    chk_rng("bytes through from idle: 16", rxn, 16, 16);
    for (int k = 0; k < 16; k++) chk8("burst byte order", rxbuf[k], 8'h60 + 8'(k));
    // Full, a place frees in the clock the shifter takes the FIFO head: a
    // write aimed at that clock takes it.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h01);
    wait_thre(DA, t_thre, st);
    for (int k = 2; k <= 17; k++) wr_reg(DA, RegRbr, 8'(k * 13));
    while (int'(cyc) != t_thre + 10 * BitClk - 1) @(negedge clk);
    chk("the load clock: a bit tick", btick[DA] === 1'b1);
    wr_now(DA, RegRbr, 8'hF1);
    chk("the head's start bit begins on it", txi[DA] === 1'b0 && txi_q[DA] === 1'b1);
    drain_rx(t_thre, t_temt);
    chk_rng("bytes through: shifter + 16 + the write in the freeing clock", rxn, 18, 18);
    chk8("that write, last", rxbuf[17], 8'hF1);
  endtask

  // Every TX FIFO slot through 00 -> FF -> 00: filled with the baud stopped,
  // then drained with each freed slot refilled.
  task automatic t_tx_sweep();
    cur  = DA;
    sect = "TX FIFO slots";
    reset_all();  // divisor 0: nothing enters the shifter
    wr_reg(DA, RegLcr, 8'h03);
    wr_reg(DA, RegIir, 8'h07);
    // With the baud disabled nothing enters the shifter: slots 0-15 fill,
    // and the seventeenth write is dropped.
    for (int k = 0; k < 17; k++) wr_reg(DA, RegRbr, 8'hFF);
    expect_reg(DA, RegLsr, 8'h00, "full");
    set_div(DA, 16'd1, 8'h03);
    wait_clk(20 * BitClk);
    for (int k = 0; k < 16; k++) begin  // one a frame, slots 0-15 in turn
      wr_reg(DA, RegRbr, 8'h00);
      wait_clk(10 * BitClk);
    end
    wait_temt(DA);
    expect_reg(DA, RegLsr, 8'h60, "drained");
  endtask

  task automatic t_rx_overrun();
    logic [7:0] v;
    cur  = DA;
    sect = "RX FIFO overrun";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'h07);
    wr_reg(DA, RegIer, 8'h05);
    for (int k = 0; k < 16; k++) rx8n1(DA, 8'h40 + 8'(k));
    expect_reg(DA, RegLsr, 8'h61, "16 held, no OE");
    rx8n1(DA, 8'h7F);
    expect_reg(DA, RegIir, 8'hC6, "17th byte: RLS");
    expect_reg(DA, RegLsr, 8'h63, "OE");
    expect_reg(DA, RegLsr, 8'h61, "OE cleared by the read");
    expect_reg(DA, RegIir, 8'hC4, "RDA");
    for (int k = 0; k < 16; k++) expect_reg(DA, RegRbr, 8'h40 + 8'(k), "the first 16 kept");
    expect_reg(DA, RegLsr, 8'h60, "the 17th was lost");
    // A byte completing into a full FIFO in the clock of an RBR read takes the
    // place that read frees: no overrun.
    for (int k = 0; k < 16; k++) rx8n1(DA, 8'h50 + 8'(k));
    fork
      rx8n1(DA, 8'h9A);
      begin
        wait_stb(DA);
        rd_now(DA, RegRbr, v);
      end
    join
    chk8("read in the completion clock", v, 8'h50);
    expect_reg(DA, RegLsr, 8'h61, "no OE: the read freed a place in that clock");
    for (int k = 1; k < 16; k++) expect_reg(DA, RegRbr, 8'h50 + 8'(k), "the rest");
    expect_reg(DA, RegRbr, 8'h9A, "the byte that completed with the read, last");
    expect_reg(DA, RegLsr, 8'h60, "empty");
  endtask

  // Errors reach LSR as their character reaches the top of the FIFO.
  task automatic t_fifo_errors();
    cur  = DA;
    sect = "FIFO line status";
    reset_all();
    set_div(DA, 16'd1, 8'h1B);  // 8E1
    wr_reg(DA, RegIir, 8'h07);
    wr_reg(DA, RegIer, 8'h05);
    rx_lcr(DA, 8'h1B, 8'h31, 1'b0, 1'b0);  // clean
    rx_lcr(DA, 8'h1B, 8'h32, 1'b1, 1'b0);  // PE
    rx_lcr(DA, 8'h1B, 8'h33, 1'b0, 1'b1);  // FE
    rx_break(DA, 12);  // BI | FE (the zero parity slot is even)
    rx_lcr(DA, 8'h1B, 8'h35, 1'b0, 1'b0);  // clean
    expect_reg(DA, RegIir, 8'hC4, "clean byte on top: RDA, no RLS yet");
    expect_reg(DA, RegLsr, 8'hE1, "clean top, errored bytes behind it: FIFOE, no PE/FE/BI");
    expect_reg(DA, RegRbr, 8'h31, "1st");
    expect_reg(DA, RegIir, 8'hC6, "PE byte on top: RLS");
    expect_reg(DA, RegLsr, 8'hE5, "PE | FIFOE before its RBR read");
    expect_reg(DA, RegLsr, 8'hE1, "PE shown once; FIFOE stays while errored bytes remain");
    expect_reg(DA, RegIir, 8'hC4, "RLS cleared");
    expect_reg(DA, RegRbr, 8'h32, "2nd (the PE byte)");
    expect_reg(DA, RegLsr, 8'hE9, "FE | FIFOE");
    expect_reg(DA, RegRbr, 8'h33, "3rd (the FE byte)");
    expect_reg(DA, RegLsr, 8'hF9, "BI | FE | FIFOE");
    expect_reg(DA, RegRbr, 8'h00, "4th (the break)");
    expect_reg(DA, RegLsr, 8'hE1, "the last errored byte read: FIFOE until an LSR read");
    expect_reg(DA, RegLsr, 8'h61, "that read cleared it");
    expect_reg(DA, RegRbr, 8'h35, "5th");
    expect_reg(DA, RegLsr, 8'h60, "empty");
    expect_reg(DA, RegIir, 8'hC1, "no interrupt");
  endtask

  // FIFOE: set while an errored character is in the FIFO, cleared by an LSR
  // read once none is.
  task automatic t_fifoe();
    logic [7:0] v;
    cur  = DA;
    sect = "FIFOE";
    reset_all();
    set_div(DA, 16'd1, 8'h1B);  // 8E1
    wr_reg(DA, RegIir, 8'h07);
    // An errored byte landing in the clock of an LSR read sets FIFOE after it.
    rx_lcr(DA, 8'h1B, 8'h21, 1'b0, 1'b0);
    fork
      rx_lcr(DA, 8'h1B, 8'h22, 1'b1, 1'b0);
      begin
        wait_stb(DA);
        rd_now(DA, RegLsr, v);
      end
    join
    chk8("LSR read as a PE byte lands: the state before it", v, 8'h61);
    expect_reg(DA, RegLsr, 8'hE1, "FIFOE from the byte that landed survives that read");
    expect_reg(DA, RegRbr, 8'h21, "clean byte");
    rd_reg(DA, RegLsr, v);  // PE on top, shown
    expect_reg(DA, RegRbr, 8'h22, "PE byte");
    // No errored byte is left, but FIFOE holds to the next LSR read; a clean
    // byte landing in that read's clock leaves it cleared.
    fork
      rx_lcr(DA, 8'h1B, 8'h23, 1'b0, 1'b0);
      begin
        wait_stb(DA);
        rd_now(DA, RegLsr, v);
      end
    join
    chk8("FIFOE still set after its byte was read", v, 8'hE0);
    expect_reg(DA, RegLsr, 8'h61, "that read cleared it");
    expect_reg(DA, RegRbr, 8'h23, "clean byte");
    // An RX FIFO clear takes the errored bytes; the next LSR read clears FIFOE.
    rx_lcr(DA, 8'h1B, 8'h24, 1'b0, 1'b0);
    rx_lcr(DA, 8'h1B, 8'h25, 1'b1, 1'b0);
    wr_reg(DA, RegIir, 8'h03);
    expect_reg(DA, RegLsr, 8'hE0, "RX FIFO cleared: FIFOE until an LSR read");
    expect_reg(DA, RegLsr, 8'h60, "then clear, no errored byte left");
    // Leaving FIFO mode clears it: D7 is 0 in 16450 mode.
    rx_lcr(DA, 8'h1B, 8'h26, 1'b1, 1'b0);
    wr_reg(DA, RegIir, 8'h00);
    rd_reg(DA, RegLsr, v);
    chk8("16450 mode: FIFOE 0", v & 8'h80, 8'h00);
  endtask

  // Every RX FIFO slot through every data and error bit: 16 x FF, 16 x break
  // under odd parity ({BI, FE, PE} = 111), 16 x 00.
  task automatic t_rx_sweep();
    logic [7:0] v;
    cur  = DA;
    sect = "RX FIFO slots";
    reset_all();
    set_div(DA, 16'd1, 8'h0B);  // 8O1
    wr_reg(DA, RegIir, 8'h07);
    for (int r = 0; r < 3; r++) begin
      for (int k = 0; k < 16; k++) begin
        if (r == 1) rx_break(DA, 12);
        else rx_lcr(DA, 8'h0B, (r == 0) ? 8'hFF : 8'h00, 1'b0, 1'b0);
      end
      for (int k = 0; k < 16; k++) begin
        rd_reg(DA, RegLsr, v);
        chk8("slot error bits", v & 8'h1F, (r == 1) ? 8'h1D : 8'h01);
        expect_reg(DA, RegRbr, (r == 0) ? 8'hFF : 8'h00, "slot data");
      end
    end
  endtask

  task automatic t_thre_rules();
    logic [7:0] v;
    int e;
    cur  = DA;
    sect = "THRE rules";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'h07);
    wr_reg(DA, RegMcr, 8'h18);  // loopback; the gate is open
    wr_reg(DA, RegIer, 8'h02);
    chk("int_o", intr[DA] === 1'b1);
    expect_reg(DA, RegIir, 8'hC2, "ETBEI with the FIFO empty: THRE");
    expect_reg(DA, RegIir, 8'hC1, "cleared by the IIR read that reported it");
    chk("int_o low", intr[DA] === 1'b0);
    wr_reg(DA, RegIer, 8'h02);
    expect_reg(DA, RegIir, 8'hC2, "an ETBEI write re-arms THRE");
    expect_reg(DA, RegIir, 8'hC1, "cleared");
    wr_reg(DA, RegIer, 8'h00);
    wr_reg(DA, RegIer, 8'h02);
    expect_reg(DA, RegIir, 8'hC2, "ETBEI 0 -> 1 re-arms THRE");
    wr_reg(DA, RegRbr, 8'hD1);
    wr_reg(DA, RegRbr, 8'hD2);
    wr_reg(DA, RegRbr, 8'hD3);
    expect_reg(DA, RegIir, 8'hC1, "THR writes clear THRE; the FIFO holds bytes");
    wait_iir(8'hC2, 40, 1'b1, 0, e);
    chk("THRE returns once the FIFO drains", e >= 0);
    rd_reg(DA, RegLsr, v);
    chk8("THRE re-raised as the last byte starts; it is still in flight", v & 8'h60, 8'h20);
    // THRE stays pending under a higher source; only its own report clears it.
    wr_reg(DA, RegIer, 8'h03);
    wait_temt(DA);
    expect_reg(DA, RegIir, 8'hC4, "RDA outranks THRE");
    expect_reg(DA, RegIir, 8'hC4, "that read did not clear THRE");
    expect_reg(DA, RegRbr, 8'hD1, "D1");
    expect_reg(DA, RegRbr, 8'hD2, "D2");
    expect_reg(DA, RegRbr, 8'hD3, "D3");
    expect_reg(DA, RegIir, 8'hC2, "THRE still pending under RDA");
    expect_reg(DA, RegIir, 8'hC1, "and now cleared");
  endtask

  // FIFO mode: THRE (LSR) rises as the last byte starts, but the THRE
  // interrupt follows a character time less the last stop bit later, unless
  // two bytes were queued at once, or FCR0 changed, since THRE last rose.
  task automatic t_thre_delay();
    int t_thre, t_int, t_temt, t2;
    bit st;
    logic [7:0] v;
    cur  = DA;
    sect = "THRE interrupt delay";
    reset_all();
    set_div(DA, 16'd1, 8'h03);  // 8N1: 10-bit frames
    wr_reg(DA, RegMcr, 8'h08);  // OUT2; ETBEI is the only enable
    wr_reg(DA, RegIer, 8'h02);
    wr_reg(DA, RegIir, 8'h07);  // FCR0 0 -> 1
    rd_reg(DA, RegIir, v);  // retire the pending THRE
    // The first after an FCR0 change: at once, a frame before TEMT.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h41);
    wait_thre(DA, t_thre, st);
    wait_int(DA, t_int);
    wait_temt_t(DA, t_temt);
    chk_rng("first after an FCR0 change: at once, a frame before TEMT", t_temt - t_int,
            10 * BitClk - 4, 10 * BitClk);
    // A lone byte: 9 bit times after THRE, in the stop bit.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h42);
    wait_thre(DA, t_thre, st);
    wait_clk(4 * BitClk);
    rd_reg(DA, RegLsr, v);
    chk8("held back: LSR THRE already set", v & 8'h60, 8'h20);
    expect_reg(DA, RegIir, 8'hC1, "held back: no THRE interrupt yet");
    wait_int(DA, t_int);
    wait_temt_t(DA, t_temt);
    chk_rng("lone byte: 9 bit times after THRE", t_int - t_thre, 9 * BitClk - 2, 9 * BitClk + 2);
    chk_rng("lone byte: one stop bit before TEMT", t_temt - t_int, BitClk - 4, BitClk);
    // Two queued at once: at once as the last starts.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h43);
    wr_reg(DA, RegRbr, 8'h44);
    wait_thre(DA, t_thre, st);
    chk("two queued: THRE as the last starts", st);
    wait_int(DA, t_int);
    wait_temt_t(DA, t_temt);
    chk_rng("two queued: at once, a frame before TEMT", t_temt - t_int, 10 * BitClk - 4,
            10 * BitClk);
    // A THR write during the hold-back cancels it; the new byte, alone in the
    // FIFO, has its own.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h45);
    wait_thre(DA, t_thre, st);
    wait_clk(4 * BitClk);
    wr_reg(DA, RegRbr, 8'h46);
    wait_clk(5 * BitClk + BitClk / 2);
    chk("a THR write during the hold-back cancels it", intr[DA] === 1'b0);
    wait_thre(DA, t2, st);
    wait_int(DA, t_int);
    chk_rng("the byte written during it: 9 bit times after its THRE", t_int - t2, 9 * BitClk - 2,
            9 * BitClk + 2);
    wait_temt(DA);
    // An ETBEI write during the hold-back raises it at once and ends the
    // hold-back: no second interrupt.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h47);
    wait_thre(DA, t_thre, st);
    wait_clk(2 * BitClk);
    wr_reg(DA, RegIer, 8'h02);
    chk("an ETBEI write with THRE set raises it", intr[DA] === 1'b1);
    expect_reg(DA, RegIir, 8'hC2, "THRE");
    wait_clk(10 * BitClk);
    chk("no second interrupt from the hold-back", intr[DA] === 1'b0);
    // A TX FIFO clear during the hold-back does the same.
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h4A);
    wait_thre(DA, t_thre, st);
    wait_clk(2 * BitClk);
    wr_reg(DA, RegIir, 8'h05);
    chk("a TX FIFO clear with THRE set raises it", intr[DA] === 1'b1);
    expect_reg(DA, RegIir, 8'hC2, "THRE");
    wait_clk(10 * BitClk);
    chk("no second interrupt after the clear", intr[DA] === 1'b0);
    // 8E2: 11 bit times, the 12-bit frame less its last stop bit.
    wr_reg(DA, RegLcr, 8'h1F);
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h48);
    wait_thre(DA, t_thre, st);
    wait_int(DA, t_int);
    wait_temt_t(DA, t_temt);
    chk_rng("8E2 lone byte: 11 bit times after THRE", t_int - t_thre, 11 * BitClk - 2,
            11 * BitClk + 2);
    chk_rng("8E2 lone byte: in the last stop bit", t_temt - t_int, BitClk - 4, BitClk);
    // FCR0 1 -> 0 -> 1: the next is at once again.
    wr_reg(DA, RegLcr, 8'h03);
    wr_reg(DA, RegIir, 8'h00);
    wr_reg(DA, RegIir, 8'h01);
    rd_reg(DA, RegIir, v);
    after_tick(DA);
    wr_reg(DA, RegRbr, 8'h49);
    wait_thre(DA, t_thre, st);
    wait_int(DA, t_int);
    wait_temt_t(DA, t_temt);
    chk_rng("first after FCR0 1 -> 0 -> 1: at once", t_temt - t_int, 10 * BitClk - 4, 10 * BitClk);
  endtask

  // All five sources pending at once, then retired one by one.
  task automatic t_iir_priority();
    int e;
    cur  = DA;
    sect = "IIR priority";
    reset_all();
    set_div(DA, 16'd1, 8'h1B);  // 8E1: 11-bit frames
    wr_reg(DA, RegIir, 8'h47);  // trigger 4
    wr_reg(DA, RegMcr, 8'h08);
    wr_reg(DA, RegIer, 8'h0F);
    cts_n[DA] = 1'b0;
    wait_clk(8);
    expect_reg(DA, RegIir, 8'hC2, "THRE outranks modem status");
    rx_lcr(DA, 8'h1B, 8'h41, 1'b1, 1'b0);  // PE
    rx_lcr(DA, 8'h1B, 8'h42, 1'b0, 1'b0);
    wait_clk(50 * BitClk);  // past the timeout
    // Each IER bit masks its own source.
    wr_reg(DA, RegIer, 8'h04);
    expect_reg(DA, RegIir, 8'hC6, "ELSI alone: RLS");
    wr_reg(DA, RegIer, 8'h01);
    expect_reg(DA, RegIir, 8'hCC, "ERBFI alone: timeout");
    wr_reg(DA, RegIer, 8'h08);
    expect_reg(DA, RegIir, 8'hC0, "EDSSI alone: modem status");
    wr_reg(DA, RegIer, 8'h02);
    expect_reg(DA, RegIir, 8'hC2, "ETBEI alone: THRE");
    wr_reg(DA, RegIer, 8'h00);
    expect_reg(DA, RegIir, 8'hC1, "IER = 0: none");
    chk("int_o low with IER = 0", intr[DA] === 1'b0);
    wr_reg(DA, RegIer, 8'h0F);
    expect_reg(DA, RegIir, 8'hC6, "all five pending: RLS first");
    chk("int_o", intr[DA] === 1'b1);
    expect_reg(DA, RegLsr, 8'hE5, "LSR: PE on the top byte");
    expect_reg(DA, RegIir, 8'hCC, "then the timeout");
    rx_lcr(DA, 8'h1B, 8'h43, 1'b0, 1'b0);
    rx_lcr(DA, 8'h1B, 8'h44, 1'b0, 1'b0);
    expect_reg(DA, RegIir, 8'hC4, "at the trigger: RDA");
    wait_clk(50 * BitClk);
    expect_reg(DA, RegIir, 8'hC4, "RDA outranks a timeout that has also fired");
    expect_reg(DA, RegRbr, 8'h41, "below the trigger again");
    expect_reg(DA, RegIir, 8'hC2, "then THRE");
    expect_reg(DA, RegIir, 8'hC0, "then modem status");
    expect_reg(DA, RegMsr, 8'h11, "MSR: CTS, DCTS");
    expect_reg(DA, RegIir, 8'hC1, "none");
    wait_iir(8'hCC, 60, 1'b0, 0, e);
    chk("the remaining bytes time out", e > 0);
    expect_reg(DA, RegRbr, 8'h42, "42");
    expect_reg(DA, RegRbr, 8'h43, "43");
    expect_reg(DA, RegRbr, 8'h44, "44");
    expect_reg(DA, RegIir, 8'hC1, "none");
    chk("int_o low", intr[DA] === 1'b0);
  endtask

  task automatic t_modem();
    logic [7:0] v;
    cur      = DA;
    sect     = "modem status";
    // Lines held asserted through reset are not reported as changes.
    rxd_drv  = '1;
    ext_loop = '0;
    cts_n    = '0;
    dsr_n    = '0;
    dcd_n    = '0;
    ri_n     = '0;
    pulse_reset();
    expect_reg(DA, RegMsr, 8'hF0, "asserted through reset: levels, no deltas");
    cts_n[DA] = 1'b1;
    dsr_n[DA] = 1'b1;
    dcd_n[DA] = 1'b1;
    ri_n[DA]  = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h0F, "all four released: DCTS DDSR TERI DDCD");
    expect_reg(DA, RegMsr, 8'h00, "deltas cleared by the read");
    // Each line alone.
    cts_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h11, "CTS asserted: CTS | DCTS");
    cts_n[DA] = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h01, "CTS released: DCTS");
    dsr_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h22, "DSR asserted: DSR | DDSR");
    dsr_n[DA] = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h02, "DSR released: DDSR");
    dcd_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h88, "DCD asserted: DCD | DDCD");
    dcd_n[DA] = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h08, "DCD released: DDCD");
    ri_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h40, "RI asserted: RI, no TERI on the leading edge");
    ri_n[DA] = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h04, "RI released: TERI");
    expect_reg(DA, RegMsr, 8'h00, "clear");
    // A delta latched in the clock MSR is read survives that read.
    @(negedge clk);
    cts_n[DA] = 1'b0;
    wait_clk(2);
    rd_now(DA, RegMsr, v);
    chk8("MSR read in the clock DCTS latches", v, 8'h00);
    expect_reg(DA, RegMsr, 8'h11, "DCTS survives it");
    cts_n[DA] = 1'b1;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'h01, "DCTS");
    // Modem-status interrupt.
    wr_reg(DA, RegIer, 8'h08);
    wr_reg(DA, RegMcr, 8'h08);
    expect_reg(DA, RegIir, 8'h01, "no delta pending");
    dcd_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegIir, 8'h00, "IIR modem status");
    chk("int_o", intr[DA] === 1'b1);
    expect_reg(DA, RegMsr, 8'h88, "MSR");
    expect_reg(DA, RegIir, 8'h01, "cleared by the MSR read");
    chk("int_o low", intr[DA] === 1'b0);
    dcd_n[DA] = 1'b1;
    wait_clk(4);
    wr_reg(DA, RegIer, 8'h00);
    expect_reg(DA, RegMsr, 8'h08, "DDCD");
  endtask

  task automatic t_loopback();
    logic [7:0] v;
    bit high;
    cur  = DA;
    sect = "loopback";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegMcr, 8'h10);
    expect_reg(DA, RegMsr, 8'h00, "loopback, MCR outputs off");
    wr_reg(DA, RegMcr, 8'h11);
    expect_reg(DA, RegMsr, 8'h22, "DTR -> DSR | DDSR");
    wr_reg(DA, RegMcr, 8'h12);
    expect_reg(DA, RegMsr, 8'h13, "RTS -> CTS | DCTS, DSR fell: DDSR");
    wr_reg(DA, RegMcr, 8'h14);
    expect_reg(DA, RegMsr, 8'h41, "OUT1 -> RI, CTS fell: DCTS; no TERI on RI rising");
    wr_reg(DA, RegMcr, 8'h18);
    expect_reg(DA, RegMsr, 8'h8C, "OUT2 -> DCD | DDCD, RI fell: TERI");
    wr_reg(DA, RegMcr, 8'h1F);
    expect_reg(DA, RegMsr, 8'hF3, "all four (OUT2 already set): DCTS DDSR");
    expect_reg(DA, RegMsr, 8'hF0, "deltas cleared");
    chk("RTS# and DTR# inactive in loopback", rts_n[DA] === 1'b1 && dtr_n[DA] === 1'b1);
    // The modem pins are disconnected from the MSR in loopback.
    cts_n[DA] = 1'b0;
    dsr_n[DA] = 1'b0;
    wait_clk(4);
    expect_reg(DA, RegMsr, 8'hF0, "pins ignored in loopback");
    // TXD holds marking while a byte loops internally.
    wr_reg(DA, RegRbr, 8'h00);
    high = 1'b1;
    for (int k = 0; k < 11 * 16; k++) begin
      wait_clk(BitClk / 16);
      if (txd[DA] !== 1'b1) high = 1'b0;
    end
    chk("TXD marking during a loopback frame", high);
    expect_reg(DA, RegLsr, 8'h61, "byte looped back");
    expect_reg(DA, RegRbr, 8'h00, "RBR");
    // Leaving loopback compares the pins with the looped values.
    wr_reg(DA, RegMcr, 8'h00);
    expect_reg(DA, RegMsr, 8'h3C, "pins CTS, DSR held; DCD and RI fell: DDCD | TERI");
    chk("RTS# and DTR# follow MCR again", rts_n[DA] === 1'b1 && dtr_n[DA] === 1'b1);
    wr_reg(DA, RegMcr, 8'h03);
    chk("RTS# and DTR# asserted", rts_n[DA] === 1'b0 && dtr_n[DA] === 1'b0);
    // int_o gating: OUT2, or loopback.
    wr_reg(DA, RegMcr, 8'h00);
    wr_reg(DA, RegIer, 8'h02);
    expect_reg(DA, RegLsr, 8'h60, "idle");
    chk("THRE pending, OUT2 clear: int_o low", intr[DA] === 1'b0);
    wr_reg(DA, RegMcr, 8'h04);
    chk("OUT1 does not open the gate", intr[DA] === 1'b0);
    wr_reg(DA, RegMcr, 8'h08);
    chk("OUT2 opens the gate", intr[DA] === 1'b1);
    wr_reg(DA, RegMcr, 8'h10);
    chk("loopback opens the gate", intr[DA] === 1'b1);
    expect_reg(DA, RegIir, 8'h02, "THRE");
    chk("int_o follows the IIR", intr[DA] === 1'b0);
  endtask

  task automatic t_baud();
    int p, c;
    logic [15:0] bits;
    bit ok;
    sect = "baud ticks";
    reset_all();
    cur = DRst;
    tick_period(DRst, 1'b1, p);
    chk_rng("RESET_DIVISOR 2: 16x tick every 2 reference periods", p, 2 * RefClk, 2 * RefClk);
    tick_period(DRst, 1'b0, p);
    chk_rng("RESET_DIVISOR 2: bit tick every 32 reference periods", p, 32 * RefClk, 32 * RefClk);
    wr_reg(DRst, RegLcr, 8'h03);
    wr_reg(DRst, RegRbr, 8'h5C);
    tx_capture(DRst, 10, 2 * BitClk, bits, ok);
    chk("transmits at the reset divisor", ok);
    chk8("frame data", bits[8:1], 8'h5C);
    chk("frame start and stop", bits[0] === 1'b0 && bits[9] === 1'b1);
    cur = DA;
    c   = 0;
    for (int k = 0; k < 2000; k++) begin
      @(negedge clk);
      if (btick[DA] || stick[DA]) c++;
    end
    chk_rng("divisor 0 out of reset: no ticks", c, 0, 0);
    set_div(DA, 16'd3, 8'h03);
    tick_period(DA, 1'b1, p);
    chk_rng("divisor 3: 16x tick every 3 reference periods", p, 3 * RefClk, 3 * RefClk);
    tick_period(DA, 1'b0, p);
    chk_rng("divisor 3: bit tick every 48 reference periods", p, 48 * RefClk, 48 * RefClk);
    set_div(DA, 16'h0100, 8'h03);
    tick_period(DA, 1'b1, p);
    chk_rng("divisor 0100 (DLM): 16x tick every 256 reference periods", p, 256 * RefClk,
            256 * RefClk);
  endtask

  task automatic t_reset_mid();
    logic [7:0] v;
    bit high;
    cur  = DA;
    sect = "reset mid-activity";
    reset_all();
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegIir, 8'hC7);
    wr_reg(DA, RegIer, 8'h0F);
    wr_reg(DA, RegMcr, 8'h0B);
    wr_reg(DA, RegScr, 8'h5A);
    ext_loop[DA] = 1'b1;
    for (int k = 0; k < 8; k++) wr_reg(DA, RegRbr, 8'hF0 + 8'(k));
    cts_n[DA] = 1'b0;
    wait_clk(25 * BitClk + 100);  // two bytes received, the third mid-frame
    expect_reg(DA, RegLsr, 8'h01, "DR, TX busy");
    chk("int_o high (modem delta)", intr[DA] === 1'b1);
    pulse_reset();
    expect_reg(DA, RegLsr, 8'h60, "LSR");
    expect_reg(DA, RegIir, 8'h01, "IIR");
    expect_reg(DA, RegIer, 8'h00, "IER");
    expect_reg(DA, RegLcr, 8'h00, "LCR");
    expect_reg(DA, RegMcr, 8'h00, "MCR");
    expect_reg(DA, RegScr, 8'h00, "SCR");
    expect_reg(DA, RegMsr, 8'h10, "MSR: CTS still held, no delta");
    chk("int_o low", intr[DA] === 1'b0);
    chk("RTS# and DTR# inactive", rts_n[DA] === 1'b1 && dtr_n[DA] === 1'b1);
    high = 1'b1;
    for (int k = 0; k < 12 * 16; k++) begin
      wait_clk(BitClk / 16);
      if (txd[DA] !== 1'b1) high = 1'b0;
    end
    chk("TXD marking: the TX FIFO and the frame in flight are gone", high);
    expect_reg(DA, RegLsr, 8'h60, "nothing received");
    set_div(DA, 16'd1, 8'h03);
    wr_reg(DA, RegRbr, 8'h3E);
    wait_dr(DA, v);
    chk8("clean frame after the reset: LSR", v & 8'h9F, 8'h01);
    expect_reg(DA, RegRbr, 8'h3E, "RBR");
    cts_n[DA] = 1'b1;
  endtask

  initial begin
    nrst = 1'b0;
    cs   = '0;
    rds  = 1'b0;
    wrs  = 1'b0;
    addr = 3'd0;
    din  = 8'h00;
    t_reset_values();
    for (int s = 0; s < 4; s++) t_registers(s);
    t_chip_select();
    t_variants();
    for (int s = 0; s < 4; s++) t_16450_mode(s);
    t_tx_wire();
    t_break();
    t_fifo_clears();
    t_triggers();
    t_timeout();
    t_tx_fill();
    t_tx_sweep();
    t_rx_overrun();
    t_fifo_errors();
    t_fifoe();
    t_rx_sweep();
    t_thre_rules();
    t_thre_delay();
    t_iir_priority();
    t_modem();
    t_loopback();
    t_baud();
    t_reset_mid();
    $display("uart16550: %0d checks, %0d mismatches", checks, errors);
    if (errors != 0) $fatal(1, "tb_uart16550 FAILED");
    $display("=== tb_uart16550: ALL CHECKS PASS ===");
    $finish;
  end

endmodule

`default_nettype wire
