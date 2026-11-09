// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_uart_rx_core -- self-checking bench for the 16x-oversampling receiver.
//
//   The bench supplies the 16x sample tick and serializes frames onto rxd_i on
//   that tick grid, 16 ticks per bit, then checks the byte, the {BI, FE, PE}
//   flags and the raw parity-slot bit each stb_o delivers.
//
//   Covers: every word length x every parity mode (none/even/odd/mark/space)
//   with data patterns that toggle every bit; parity, framing and break
//   errors in every combination the flag logic distinguishes (a zero byte
//   with a 1 in the parity slot is a framing error, not a break); a start-bit
//   glitch shorter than half a bit, rejected without a strobe; the line still
//   low after a framing error not re-triggering a start; a fractional
//   clocks-per-tick grid (the tick spacing alternating 4 and 5 clocks); a
//   reset mid-frame; and the state register forced to an unused code, which
//   the FSM's default arm must return to idle (no input sequence reaches those
//   codes: see sim/formal/uart_cores_formal.sv).
//============================================================================
`default_nettype none

module tb_uart_rx_core;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic nrst = 1'b0;

  // 16x sample tick: one pulse every `period` clocks; with `jitter` set the
  // spacing alternates 4 and 5 clocks (a fractional clocks-per-bit ratio).
  logic samp16 = 1'b0;
  int   period = 4;
  logic jitter = 1'b0;
  int   cur_period = 4;
  int   tp_cnt = 0;
  always @(posedge clk) begin
    if (tp_cnt >= cur_period - 1) begin
      samp16     <= 1'b1;
      tp_cnt     <= 0;
      cur_period <= jitter ? ((cur_period == 4) ? 5 : 4) : period;
    end else begin
      samp16 <= 1'b0;
      tp_cnt <= tp_cnt + 1;
    end
  end

  logic rxd = 1'b1;
  logic [1:0] word_len;
  logic parity_en, parity_even, parity_stick;
  logic [7:0] data;
  logic [2:0] err;
  logic pbit_o, stb;

  uart_rx_core dut (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .samp16_tick_i (samp16),
      .rxd_i         (rxd),
      .word_len_i    (word_len),
      .parity_en_i   (parity_en),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .data_o        (data),
      .err_o         (err),
      .parity_bit_o  (pbit_o),
      .stb_o         (stb)
  );

  integer errors = 0;
  task automatic check(input logic cond, input string what);
    begin
      if (cond !== 1'b1) begin
        $display("FAIL: %s", what);
        errors = errors + 1;
      end
    end
  endtask

  // Strobe monitor: count strobes, keep the last delivery, and make sure stb_o
  // is a single-cycle pulse.
  int stb_count = 0;
  logic [7:0] last_data;
  logic [2:0] last_err;
  logic last_pb;
  logic stb_d1 = 1'b0;
  always @(posedge clk) begin
    stb_d1 <= stb;
    if (stb) begin
      stb_count <= stb_count + 1;
      last_data <= data;
      last_err  <= err;
      last_pb   <= pbit_o;
      if (stb_d1) begin
        $display("FAIL: stb_o high for two consecutive clocks");
        errors = errors + 1;
      end
    end
  end

  localparam int PNone = 0;
  localparam int PEven = 1;
  localparam int POdd = 2;
  localparam int PMark = 3;
  localparam int PSpace = 4;

  function automatic string pname(input int p);
    case (p)
      PEven: return "E";
      POdd: return "O";
      PMark: return "M";
      PSpace: return "S";
      default: return "N";
    endcase
  endfunction

  task automatic set_frame(input int wl, input int par);
    begin
      word_len     = wl[1:0];
      parity_en    = (par != PNone);
      parity_stick = (par == PMark) || (par == PSpace);
      parity_even  = (par == PEven) || (par == PSpace);
    end
  endtask

  function automatic int ndata_of(input int wl);
    return 5 + wl;
  endfunction

  function automatic logic [7:0] masked(input logic [7:0] b, input int wl);
    return b & (8'hFF >> (8 - ndata_of(wl)));
  endfunction

  function automatic logic exp_parity(input logic [7:0] b, input int wl, input int par);
    logic x;
    x = ^masked(b, wl);
    case (par)
      PEven: return x;
      POdd: return ~x;
      PMark: return 1'b1;
      PSpace: return 1'b0;
      default: return 1'b0;
    endcase
  endfunction

  task automatic ticks(input int n);
    repeat (n) @(posedge samp16);
  endtask

  // Serialize one frame: start, nd data LSB-first, the parity slot (when the
  // config has one) carrying `pslot`, the stop bit at `stop_lvl`, then idle.
  task automatic send_frame(input logic [7:0] b, input int wl, input int par, input logic pslot,
                            input logic stop_lvl, input int idle_ticks);
    begin
      @(posedge samp16);
      rxd = 1'b0;
      ticks(16);
      for (int i = 0; i < ndata_of(wl); i++) begin
        rxd = b[i];
        ticks(16);
      end
      if (par != PNone) begin
        rxd = pslot;
        ticks(16);
      end
      rxd = stop_lvl;
      ticks(16);
      rxd = 1'b1;
      ticks(idle_ticks);
    end
  endtask

  // Send a frame and check exactly one delivery with the expected fields.
  task automatic xfer(input logic [7:0] b, input int wl, input int par, input logic pslot,
                      input logic stop_lvl, input logic [2:0] exp_err, input string why);
    int n0;
    logic [7:0] exp;
    string tag;
    begin
      n0 = stb_count;
      tag = $sformatf("%0d%s byte %02h pslot %b stop %b (%s)", ndata_of(wl), pname(par), b, pslot,
                      stop_lvl, why);
      send_frame(b, wl, par, pslot, stop_lvl, 8);
      exp = masked(b, wl);
      check(stb_count == n0 + 1, $sformatf("%s: one strobe (got %0d)", tag, stb_count - n0));
      check(last_data === exp, $sformatf("%s: data (exp %02h got %02h)", tag, exp, last_data));
      check(last_err === exp_err, $sformatf("%s: err (exp %b got %b)", tag, exp_err, last_err));
      check(last_pb === ((par != PNone) ? pslot : 1'b0), $sformatf(
            "%s: parity_bit_o %b", tag, last_pb));
    end
  endtask

  // A good frame: the parity slot carries the right bit, the stop bit is high.
  task automatic good(input logic [7:0] b, input int wl, input int par);
    xfer(b, wl, par, exp_parity(b, wl, par), 1'b1, 3'b000, "good");
  endtask

  logic [7:0] patterns[6];
  int n0;

  initial begin
    set_frame(3, PNone);
    nrst = 1'b0;
    repeat (4) @(posedge clk);
    nrst = 1'b1;
    repeat (4) @(posedge clk);

    $display("--- tb_uart_rx_core ---");
    check(stb === 1'b0 && err === 3'b000, "quiet after reset");

    // Test 1: the frame matrix, every word length and parity mode.
    patterns[0] = 8'h00;
    patterns[1] = 8'hFF;
    patterns[2] = 8'h55;
    patterns[3] = 8'hAA;
    patterns[4] = 8'h81;
    patterns[5] = 8'h7E;
    for (int wl = 0; wl < 4; wl++) begin
      for (int par = PNone; par <= PSpace; par++) begin
        set_frame(wl, par);
        for (int k = 0; k < 6; k++) good(patterns[k], wl, par);
        good(8'($urandom), wl, par);
      end
    end

    // Test 2: error flags.
    set_frame(3, PEven);
    xfer(8'h55, 3, PEven, ~exp_parity(8'h55, 3, PEven), 1'b1, 3'b001, "parity error");
    xfer(8'h55, 3, PEven, exp_parity(8'h55, 3, PEven), 1'b0, 3'b010, "framing error");
    good(8'h66, 3, PEven);  // the line was low into the stop: next frame still lands
    xfer(8'h00, 3, PEven, 1'b0, 1'b0, 3'b110, "break with an even-parity slot");
    good(8'h99, 3, PEven);
    set_frame(3, POdd);
    xfer(8'h00, 3, POdd, 1'b1, 1'b0, 3'b010, "zero byte, parity slot 1, stop low: FE not BI");
    xfer(8'h00, 3, POdd, 1'b1, 1'b1, 3'b000, "zero byte with odd parity: clean");
    set_frame(3, PMark);
    xfer(8'h00, 3, PMark, 1'b0, 1'b0, 3'b111, "all flags: mark parity read 0, stop low");
    set_frame(3, PNone);
    xfer(8'h00, 3, PNone, 1'b0, 1'b1, 3'b000, "zero byte, stop high: not a break");
    xfer(8'h00, 3, PNone, 1'b0, 1'b0, 3'b110, "break, no parity");
    xfer(8'h80, 3, PNone, 1'b0, 1'b0, 3'b010, "framing error, data not all zero");
    set_frame(0, PSpace);
    xfer(8'h00, 0, PSpace, 1'b1, 1'b0, 3'b011, "5-bit: PE + FE, slot 1 clears the break track");
    good(8'h1F, 0, PSpace);

    // Test 3: a long break (the line low for several frame times) delivers one
    // break character, then nothing until the line rises and a start follows.
    set_frame(3, PNone);
    n0 = stb_count;
    @(posedge samp16);
    rxd = 1'b0;
    ticks(16 * 10 * 3);
    rxd = 1'b1;
    ticks(32);
    check(stb_count == n0 + 1, $sformatf("long break: one strobe (got %0d)", stb_count - n0));
    check(last_err === 3'b110 && last_data === 8'h00, "long break: BI+FE, zero byte");
    good(8'hA5, 3, PNone);

    // Test 4: a start-bit glitch shorter than half a bit is rejected.
    n0 = stb_count;
    @(posedge samp16);
    rxd = 1'b0;
    ticks(3);
    rxd = 1'b1;
    ticks(16 * 12);
    check(stb_count == n0, "glitch: no strobe");
    good(8'h3C, 3, PNone);

    // Test 5: fractional clocks-per-tick (4/5 alternating): the 16x grid is
    // what the receiver counts, so every frame still lands.
    jitter = 1'b1;
    set_frame(3, PNone);
    for (int k = 0; k < 6; k++) good(patterns[k], 3, PNone);
    set_frame(2, PEven);
    for (int k = 0; k < 6; k++) good(patterns[k], 2, PEven);
    jitter = 1'b0;
    repeat (16) @(posedge samp16);

    // Test 6: reset mid-frame: no strobe for the aborted frame, and the next
    // frame is received cleanly.
    set_frame(3, POdd);
    n0 = stb_count;
    fork
      send_frame(8'hC3, 3, POdd, exp_parity(8'hC3, 3, POdd), 1'b1, 8);
      begin
        ticks(16 * 4);  // into data bit 2
        @(negedge clk);
        nrst = 1'b0;  // held until the aborted frame has left the line
      end
    join
    check(stb_count == n0, "reset mid-frame: no strobe");
    check(rxd === 1'b1, "reset mid-frame: line idle before reset release");
    @(negedge clk);
    nrst = 1'b1;
    ticks(8);
    check(data === 8'h00 && err === 3'b000, "reset mid-frame: outputs cleared");
    good(8'h5A, 3, POdd);

    // Test 7: fault injection. Force the state register to each unused code
    // (5..7) for a clock; the default arm returns the FSM to idle on the next
    // clock, with no strobe, and the next frame is received cleanly.
    n0 = stb_count;
    for (int code = 5; code <= 7; code++) begin
      @(negedge clk);
      force dut.state_q = code[2:0];
      @(negedge clk);
      release dut.state_q;
      check(dut.state_q === code[2:0], $sformatf("fault %0d: code held after release", code));
      @(negedge clk);
      check(dut.state_q === 3'd0, $sformatf("fault %0d: default arm returned to idle", code));
    end
    ticks(4);
    check(stb_count == n0, "fault injection: no strobe");
    good(8'hA5, 3, POdd);

    if (errors == 0) $display("=== tb_uart_rx_core: ALL CHECKS PASS ===");
    else $display("=== tb_uart_rx_core: %0d FAILURE(S) ===", errors);
    if (errors != 0) $fatal(1, "tb_uart_rx_core FAILED");
    $finish;
  end

  initial begin
    #20_000_000;
    $display("WATCHDOG timeout");
    $fatal(1, "tb_uart_rx_core watchdog");
  end

endmodule

`default_nettype wire
