// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_uart_tx_core -- self-checking bench for the configurable-frame transmitter.
//
//   The bench supplies the per-bit baud tick (one pulse every TICK clocks) and
//   decodes txd_o bit by bit at mid-bit, so every frame the core emits is
//   checked bit for bit: start, n data LSB-first, the parity bit the config
//   calls for, and 1 or 2 stop bits each a full bit period long.
//
//   Covers: every word length x every parity mode (none/even/odd/mark/space) x
//   1/2 stop bits, with data patterns that toggle every bit; the holding
//   buffer (ready_o/tx_idle_o, a byte queued behind a frame in flight starts
//   exactly after the stop bits, a write while the buffer is full is dropped);
//   word length and parity latched at load; break_i forcing the line low;
//   a reset mid-frame; and the
//   state register forced to an unused code, which the FSM's default arm
//   must return to idle (no input sequence reaches those codes: see
//   sim/formal/uart_cores_formal.sv). A second instance without the holding
//   buffer (HOLD = 0) on the same tick and config: ready_o only in the load
//   clock, a we_i outside it ignored, a byte offered in it starting its frame
//   on that tick, and the next one offered at the last stop bit's end
//   following back to back.
//============================================================================
`default_nettype none

module tb_uart_tx_core;

  localparam int TICK = 8;  // clocks per bit period

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic nrst = 1'b0;

  // Baud tick: one clock-wide pulse every TICK clocks.
  logic baud_tick = 1'b0;
  int   tick_cnt = 0;
  always @(posedge clk) begin
    baud_tick <= (tick_cnt == TICK - 1);
    tick_cnt  <= (tick_cnt == TICK - 1) ? 0 : tick_cnt + 1;
  end

  logic [7:0] data;
  logic we;
  logic [1:0] word_len;
  logic parity_en, parity_even, parity_stick, stop2, brk;
  logic ready, tx_idle, txd;

  uart_tx_core dut (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .baud_tick_i   (baud_tick),
      .data_i        (data),
      .we_i          (we),
      .word_len_i    (word_len),
      .parity_en_i   (parity_en),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .stop2_i       (stop2),
      .break_i       (brk),
      .ready_o       (ready),
      .tx_idle_o     (tx_idle),
      .txd_o         (txd)
  );

  // The build without the holding buffer.
  logic [7:0] data0;
  logic we0, ready0, tx_idle0, txd0;

  uart_tx_core #(
      .HOLD(1'b0)
  ) dut0 (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .baud_tick_i   (baud_tick),
      .data_i        (data0),
      .we_i          (we0),
      .word_len_i    (word_len),
      .parity_en_i   (parity_en),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .stop2_i       (stop2),
      .break_i       (brk),
      .ready_o       (ready0),
      .tx_idle_o     (tx_idle0),
      .txd_o         (txd0)
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

  task automatic check_marking(input int clocks);
    // Observe every stable cycle; looking only at the end can miss a whole
    // unintended frame that starts and finishes inside the idle window.
    repeat (clocks) begin
      @(negedge clk);
      check(txd === 1'b1, "refused write: no second frame");
    end
  endtask

  // Parity modes.
  localparam int PNone = 0;
  localparam int PEven = 1;
  localparam int POdd = 2;
  localparam int PMark = 3;  // stick, parity_even 0 -> bit 1
  localparam int PSpace = 4;  // stick, parity_even 1 -> bit 0

  function automatic string pname(input int p);
    case (p)
      PEven: return "E";
      POdd: return "O";
      PMark: return "M";
      PSpace: return "S";
      default: return "N";
    endcase
  endfunction

  task automatic set_frame(input int wl, input int par, input logic s2);
    begin
      word_len     = wl[1:0];
      parity_en    = (par != PNone);
      parity_stick = (par == PMark) || (par == PSpace);
      parity_even  = (par == PEven) || (par == PSpace);
      stop2        = s2;
    end
  endtask

  function automatic int ndata_of(input int wl);
    return 5 + wl;
  endfunction

  function automatic logic exp_parity(input logic [7:0] b, input int wl, input int par);
    logic [7:0] m;
    logic x;
    m = b & (8'hFF >> (8 - ndata_of(wl)));
    x = ^m;
    case (par)
      PEven: return x;
      POdd: return ~x;
      PMark: return 1'b1;
      PSpace: return 1'b0;
      default: return 1'b0;
    endcase
  endfunction

  // Enqueue one byte (one-cycle we_i).
  task automatic enqueue(input logic [7:0] b);
    begin
      @(negedge clk);
      data = b;
      we   = 1'b1;
      @(negedge clk);
      we = 1'b0;
    end
  endtask

  // Decode one frame off txd: wait for the start edge, then sample each bit
  // period at its middle. Returns the data (right-aligned), the parity bit
  // seen (if any), and the stop-bit levels. Also reports the start time.
  task automatic recv_frame(input int wl, input int par, input logic s2, output logic [7:0] d,
                            output logic pbit, output logic stops_ok, output time t_start);
    int nd;
    begin
      nd       = ndata_of(wl);
      d        = 8'h00;
      pbit     = 1'b0;
      stops_ok = 1'b1;
      @(negedge txd);
      t_start = $time;
      repeat (TICK / 2) @(posedge clk);  // mid-start
      check(txd === 1'b0, "start bit low at mid-bit");
      for (int i = 0; i < nd; i++) begin
        repeat (TICK) @(posedge clk);
        d[i] = txd;
      end
      if (par != PNone) begin
        repeat (TICK) @(posedge clk);
        pbit = txd;
      end
      repeat (TICK) @(posedge clk);
      if (txd !== 1'b1) stops_ok = 1'b0;
      if (s2) begin
        repeat (TICK) @(posedge clk);
        if (txd !== 1'b1) stops_ok = 1'b0;
      end
    end
  endtask

  // HOLD = 0: offer a byte in the next load clock; returns at the negedge
  // after it, with the time the start bit began.
  task automatic offer0(input logic [7:0] b, output time t_start);
    begin
      do @(negedge clk); while (ready0 !== 1'b1);
      data0 = b;
      we0   = 1'b1;
      @(negedge clk);
      we0     = 1'b0;
      t_start = $time;
      check(txd0 === 1'b0, $sformatf("HOLD 0: byte %02h's start bit begins on the tick", b));
    end
  endtask

  // HOLD = 0: decode an 8N1 frame whose start bit began at the last negedge.
  task automatic recv0(output logic [7:0] d, output logic stop_ok);
    begin
      repeat (TICK / 2) @(posedge clk);  // mid-start
      check(txd0 === 1'b0, "HOLD 0: start bit low at mid-bit");
      for (int i = 0; i < 8; i++) begin
        repeat (TICK) @(posedge clk);
        d[i] = txd0;
      end
      repeat (TICK) @(posedge clk);
      stop_ok = (txd0 === 1'b1);
    end
  endtask

  // Send one byte and check the frame on the wire against the config.
  task automatic xfer(input logic [7:0] b, input int wl, input int par, input logic s2);
    logic [7:0] d, exp;
    logic pbit, sok;
    time   t0;
    string tag;
    begin
      tag = $sformatf("%0d%s%0d byte %02h", ndata_of(wl), pname(par), s2 ? 2 : 1, b);
      check(ready === 1'b1, {tag, ": ready before enqueue"});
      fork
        enqueue(b);
        recv_frame(wl, par, s2, d, pbit, sok, t0);
      join
      exp = b & (8'hFF >> (8 - ndata_of(wl)));
      check(d === exp, $sformatf("%s: data (exp %02h got %02h)", tag, exp, d));
      if (par != PNone)
        check(pbit === exp_parity(b, wl, par), $sformatf("%s: parity bit %b", tag, pbit));
      check(sok, {tag, ": stop bit(s) high"});
      // Let the frame finish and the core go idle.
      repeat (2 * TICK) @(posedge clk);
      check(tx_idle === 1'b1, {tag, ": tx_idle after the frame"});
      check(txd === 1'b1, {tag, ": line marking after the frame"});
    end
  endtask

  logic [7:0] patterns[6];
  logic [7:0] d, d2;
  logic pb, sok;
  time t0, t1;
  int frame_bits;

  initial begin
    data  = 8'h00;
    we    = 1'b0;
    data0 = 8'h00;
    we0   = 1'b0;
    brk   = 1'b0;
    set_frame(3, PNone, 1'b0);
    nrst = 1'b0;
    repeat (4) @(posedge clk);
    nrst = 1'b1;
    repeat (4) @(posedge clk);

    $display("--- tb_uart_tx_core ---");

    // Reset state.
    check(txd === 1'b1, "txd idles high");
    check(ready === 1'b1, "ready after reset");
    check(tx_idle === 1'b1, "tx_idle after reset");

    // Test 1: the frame matrix.
    patterns[0] = 8'h00;
    patterns[1] = 8'hFF;
    patterns[2] = 8'h55;
    patterns[3] = 8'hAA;
    patterns[4] = 8'h81;
    patterns[5] = 8'h7E;
    for (int wl = 0; wl < 4; wl++) begin
      for (int par = PNone; par <= PSpace; par++) begin
        for (int s2 = 0; s2 < 2; s2++) begin
          set_frame(wl, par, s2[0]);
          for (int k = 0; k < 6; k++) xfer(patterns[k], wl, par, s2[0]);
          xfer(8'($urandom), wl, par, s2[0]);
        end
      end
    end

    // Test 2: a byte queued behind a frame in flight starts exactly after the
    // stop bit(s); ready_o drops while the buffer holds it; tx_idle stays low.
    for (int s2 = 0; s2 < 2; s2++) begin
      set_frame(3, PEven, s2[0]);
      frame_bits = 1 + 8 + 1 + (s2 ? 2 : 1);
      fork
        begin
          enqueue(8'h3C);
          // ready returns high once the byte loads into the shifter.
          do @(posedge clk); while (!ready);
          check(tx_idle === 1'b0, "tx_idle low with a frame in flight");
          enqueue(8'hC3);
          @(negedge clk);
          check(ready === 1'b0, "ready low while the holding buffer is full");
        end
        begin
          recv_frame(3, PEven, s2[0], d, pb, sok, t0);
          check(d === 8'h3C && sok, "back-to-back: first frame");
          recv_frame(3, PEven, s2[0], d2, pb, sok, t1);
          check(d2 === 8'hC3 && sok, "back-to-back: second frame");
          check((t1 - t0) == frame_bits * TICK * 10, $sformatf(
                "back-to-back: second start %0d ns after the first (exp %0d)",
                t1 - t0,
                frame_bits * TICK * 10
                ));
        end
      join
      repeat (2 * TICK) @(posedge clk);
      check(tx_idle === 1'b1, "tx_idle after the back-to-back pair");
    end

    // Test 3: a write while the holding buffer is full is dropped.
    set_frame(3, PNone, 1'b0);
    @(posedge baud_tick);  // a full bit period before the next tick can load
    fork
      begin
        enqueue(8'h5A);
        enqueue(8'hA5);  // ready is low: refused
      end
      begin
        recv_frame(3, PNone, 1'b0, d, pb, sok, t0);
        check(d === 8'h5A, "dropped write: the first byte went out");
      end
    join
    // No second frame: the line stays marking for a frame time.
    check_marking(12 * TICK);
    check(tx_idle === 1'b1, "dropped write: transmitter is idle");

    // Test 4: the config is latched at load. Change word length and parity
    // mid-frame; the frame in flight keeps its 5-bit odd framing.
    set_frame(0, POdd, 1'b0);
    fork
      begin
        enqueue(8'h1B);
        repeat (3 * TICK) @(posedge clk);  // into the data bits
        set_frame(3, PNone, 1'b0);
      end
      begin
        recv_frame(0, POdd, 1'b0, d, pb, sok, t0);
        check(d === 8'h1B, "latched config: 5 data bits");
        check(pb === exp_parity(8'h1B, 0, POdd), "latched config: odd parity bit");
        check(sok, "latched config: stop bit");
      end
    join
    repeat (2 * TICK) @(posedge clk);

    // Test 5: break. Idle: the line drops at once and returns on release.
    set_frame(3, PNone, 1'b0);
    @(negedge clk);
    brk = 1'b1;
    @(negedge clk);
    check(txd === 1'b0, "break forces txd low while idle");
    repeat (2 * TICK) @(posedge clk);
    @(negedge clk);
    brk = 1'b0;
    @(negedge clk);
    check(txd === 1'b1, "line returns to marking after break");
    // Mid-frame: the line is held low through the data bits, the frame still
    // completes, and the core is idle afterwards.
    begin
      int high_seen;
      high_seen = 0;
      enqueue(8'hFF);
      @(negedge txd);  // start bit
      repeat (TICK + TICK / 2) @(posedge clk);  // into data bit 0 (a 1)
      check(txd === 1'b1, "break mid-frame: data bit 0 high before break");
      @(negedge clk);
      brk = 1'b1;
      repeat (4 * TICK) @(posedge clk);
      if (txd !== 1'b0) high_seen = 1;
      repeat (4 * TICK) @(posedge clk);
      if (txd !== 1'b0) high_seen = 1;
      check(!high_seen, "break mid-frame: line low for the rest of the frame");
      @(negedge clk);
      brk = 1'b0;
      repeat (4 * TICK) @(posedge clk);
      check(tx_idle === 1'b1 && txd === 1'b1, "break mid-frame: idle after release");
    end

    // Test 6: reset mid-frame returns the core to idle at once.
    set_frame(3, PEven, 1'b1);
    enqueue(8'h00);  // data bits all 0: the line is low through them
    @(negedge txd);
    repeat (3 * TICK) @(posedge clk);
    check(txd === 1'b0, "reset mid-frame: line low in the data bits");
    @(negedge clk);
    nrst = 1'b0;
    @(negedge clk);
    check(txd === 1'b1 && ready === 1'b1 && tx_idle === 1'b1, "reset mid-frame: idle and marking");
    repeat (2) @(negedge clk);
    nrst = 1'b1;
    repeat (4 * TICK) @(posedge clk);
    check(txd === 1'b1 && tx_idle === 1'b1, "stays idle after reset release");
    xfer(8'h96, 3, PEven, 1'b1);

    // Test 7: fault injection. Force the state register to each unused code
    // (1, 5..7) across one baud tick; the default arm returns the FSM to idle on
    // the next tick and the core transmits normally afterwards.
    for (int code = 1; code <= 7; code++) begin
      if (code >= 2 && code <= 4) continue;
      @(negedge clk);
      force dut.state_q = code[2:0];
      @(posedge baud_tick);
      @(negedge clk);
      release dut.state_q;
      check(dut.state_q === code[2:0], $sformatf("fault %0d: code held after release", code));
      @(posedge baud_tick);
      @(negedge clk);
      check(dut.state_q === 3'd0, $sformatf("fault %0d: default arm returned to idle", code));
      check(tx_idle === 1'b1 && txd === 1'b1, $sformatf("fault %0d: idle and marking", code));
    end
    set_frame(3, PNone, 1'b0);
    xfer(8'h69, 3, PNone, 1'b0);

    // Test 8: no holding buffer (HOLD = 0). Idle, ready_o is the baud tick.
    begin
      int   bad;
      logic sok0;
      bad = 0;
      repeat (3 * TICK) begin
        @(negedge clk);
        if (ready0 !== baud_tick) bad++;
      end
      check(bad == 0 && tx_idle0 === 1'b1, "HOLD 0 idle: ready_o is the baud tick");
      // A we_i outside the load clock is ignored.
      @(posedge baud_tick);
      repeat (2) @(negedge clk);  // the clock after the tick
      check(ready0 === 1'b0, "HOLD 0: ready_o low off the tick");
      data0 = 8'hA6;
      we0   = 1'b1;
      @(negedge clk);
      we0 = 1'b0;
      repeat (2 * TICK) @(negedge clk);
      check(txd0 === 1'b1 && tx_idle0 === 1'b1, "HOLD 0: a we_i off the load clock is ignored");
      // Two bytes back to back: the second offered at the first's stop end.
      bad = 0;
      offer0(8'h3C, t0);
      check(tx_idle0 === 1'b0, "HOLD 0: tx_idle low with a frame in flight");
      fork
        begin
          recv0(d, sok0);
          check(d === 8'h3C && sok0, "HOLD 0: first frame");
        end
        begin
          // ready_o stays low through the frame until its last stop bit ends.
          repeat (10 * TICK - 2) begin
            @(negedge clk);
            if (ready0 !== 1'b0) bad++;
          end
        end
      join
      check(bad == 0, "HOLD 0: ready_o low through the frame");
      offer0(8'hC3, t1);
      check((t1 - t0) == 10 * TICK * 10, $sformatf(
            "HOLD 0: second start %0d ns after the first (exp %0d)", t1 - t0, 10 * TICK * 10));
      recv0(d2, sok0);
      check(d2 === 8'hC3 && sok0, "HOLD 0: second frame");
      repeat (2 * TICK) @(posedge clk);
      check(tx_idle0 === 1'b1 && txd0 === 1'b1, "HOLD 0: idle after the pair");
    end

    if (errors == 0) $display("=== tb_uart_tx_core: ALL CHECKS PASS ===");
    else $display("=== tb_uart_tx_core: %0d FAILURE(S) ===", errors);
    if (errors != 0) $fatal(1, "tb_uart_tx_core FAILED");
    $finish;
  end

  initial begin
    #5_000_000;
    $display("WATCHDOG timeout");
    $fatal(1, "tb_uart_tx_core watchdog");
  end

endmodule

`default_nettype wire
