// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_uart_baud_dds -- self-checking bench for the exact-baud generator.
//
//   Four instances: `u_std` at the default 48 MHz / 1.8432 MHz (a fractional
//   26.04 clocks per reference tick, so the DDS stage does real work);
//   `u_fast` with CLK_HZ == REF_HZ (a reference tick every clock), which makes
//   the divider stage's arithmetic observable clock for clock and keeps the
//   65535-divisor test short; `u_hs` at 200 MHz (108.5 clocks per tick, the
//   accumulator spanning 2^27); and `u_cb` at the 3.579545 MHz colorburst
//   clock, an odd count that is under two clocks per tick and walks every low
//   accumulator bit.
//
//   Covers: divisor 0 disabling both ticks; the exact average tick rate at
//   divisors 1, 3 and 12 over a window that is a whole number of reference
//   periods (480,000 clocks = 18,432 reference ticks); the /16 relation
//   between the two ticks; the full 16-bit divisor range (65535 clocks per
//   tick); a divisor lowered below the running count firing at once (the
//   compare is >=, not ==); resync_i restarting the bit phase so the next
//   bit tick is exactly 16 sample ticks later; and a reset mid-run.
//============================================================================
`default_nettype none

module tb_uart_baud_dds;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic nrst = 1'b0;

  logic [15:0] div_std, div_fast, div_x;
  logic resync_std, resync_fast;
  logic s16_std, bit_std, s16_fast, bit_fast, s16_hs, bit_hs, s16_cb, bit_cb;

  uart_baud_dds #(
      .CLK_HZ(48_000_000),
      .REF_HZ(1_843_200)
  ) u_std (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (div_std),
      .resync_i     (resync_std),
      .samp16_tick_o(s16_std),
      .bit_tick_o   (bit_std)
  );

  uart_baud_dds #(
      .CLK_HZ(1_843_200),
      .REF_HZ(1_843_200)
  ) u_fast (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (div_fast),
      .resync_i     (resync_fast),
      .samp16_tick_o(s16_fast),
      .bit_tick_o   (bit_fast)
  );

  uart_baud_dds #(
      .CLK_HZ(200_000_000),
      .REF_HZ(1_843_200)
  ) u_hs (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (div_x),
      .resync_i     (1'b0),
      .samp16_tick_o(s16_hs),
      .bit_tick_o   (bit_hs)
  );

  uart_baud_dds #(
      .CLK_HZ(3_579_545),
      .REF_HZ(1_843_200)
  ) u_cb (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (div_x),
      .resync_i     (1'b0),
      .samp16_tick_o(s16_cb),
      .bit_tick_o   (bit_cb)
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

  // Tick counters (sampled at the clock edge that sees each pulse).
  int n_s16_std = 0, n_bit_std = 0, n_s16_fast = 0, n_bit_fast = 0;
  int n_s16_hs = 0, n_bit_hs = 0, n_s16_cb = 0, n_bit_cb = 0;
  always @(posedge clk) begin
    if (s16_std) n_s16_std <= n_s16_std + 1;
    if (bit_std) n_bit_std <= n_bit_std + 1;
    if (s16_fast) n_s16_fast <= n_s16_fast + 1;
    if (bit_fast) n_bit_fast <= n_bit_fast + 1;
    if (s16_hs) n_s16_hs <= n_s16_hs + 1;
    if (bit_hs) n_bit_hs <= n_bit_hs + 1;
    if (s16_cb) n_s16_cb <= n_s16_cb + 1;
    if (bit_cb) n_bit_cb <= n_bit_cb + 1;
  end

  // Count u_std's ticks over `clocks` clocks at divisor `d`; expect `exp`
  // sample ticks within a two-tick window-boundary slop, and the bit tick at
  // one sixteenth of them.
  task automatic rate_std(input logic [15:0] d, input int clocks, input int exp);
    int s0, b0, ds, db;
    begin
      @(negedge clk);
      div_std = d;
      repeat (64) @(posedge clk);  // settle the divider onto the new divisor
      s0 = n_s16_std;
      b0 = n_bit_std;
      repeat (clocks) @(posedge clk);
      ds = n_s16_std - s0;
      db = n_bit_std - b0;
      check(ds >= exp - 2 && ds <= exp + 2, $sformatf(
            "div %0d: %0d samp16 ticks in %0d clocks (exp %0d)", d, ds, clocks, exp));
      check(db * 16 >= ds - 16 && db * 16 <= ds + 16, $sformatf(
            "div %0d: %0d bit ticks for %0d samp16 ticks", d, db, ds));
    end
  endtask

  // The same rate check on u_hs and u_cb, which share div_x; `clocks` is their
  // window in their own clock, so the expectation is per instance.
  task automatic rate_x(input logic [15:0] d, input int clocks_hs, input int exp_hs,
                        input int clocks_cb, input int exp_cb);
    int s0, b0, ds, db;
    begin
      @(negedge clk);
      div_x = d;
      repeat (64) @(posedge clk);
      s0 = n_s16_hs;
      b0 = n_bit_hs;
      repeat (clocks_hs) @(posedge clk);
      ds = n_s16_hs - s0;
      db = n_bit_hs - b0;
      check(ds >= exp_hs - 2 && ds <= exp_hs + 2, $sformatf(
            "200 MHz div %0d: %0d samp16 ticks in %0d clocks (exp %0d)", d, ds, clocks_hs, exp_hs));
      check(db * 16 >= ds - 16 && db * 16 <= ds + 16, $sformatf(
            "200 MHz div %0d: %0d bit ticks for %0d samp16 ticks", d, db, ds));
      s0 = n_s16_cb;
      b0 = n_bit_cb;
      repeat (clocks_cb) @(posedge clk);
      ds = n_s16_cb - s0;
      db = n_bit_cb - b0;
      check(ds >= exp_cb - 2 && ds <= exp_cb + 2, $sformatf(
            "3.58 MHz div %0d: %0d samp16 ticks in %0d clocks (exp %0d)", d, ds, clocks_cb, exp_cb
            ));
      check(db * 16 >= ds - 16 && db * 16 <= ds + 16, $sformatf(
            "3.58 MHz div %0d: %0d bit ticks for %0d samp16 ticks", d, db, ds));
    end
  endtask

  // Clocks between two consecutive u_fast sample ticks (bit ticks with `bit_`).
  task automatic period_fast(output int clocks);
    time t0;
    begin
      @(posedge s16_fast);
      t0 = $time;
      @(posedge s16_fast);
      clocks = int'(($time - t0) / 10);
    end
  endtask

  task automatic bit_period_fast(output int clocks);
    time t0;
    begin
      @(posedge bit_fast);
      t0 = $time;
      @(posedge bit_fast);
      clocks = int'(($time - t0) / 10);
    end
  endtask

  int p, n, s0, b0;

  initial begin
    div_std     = 16'd0;
    div_fast    = 16'd0;
    div_x       = 16'd0;
    resync_std  = 1'b0;
    resync_fast = 1'b0;
    nrst        = 1'b0;
    repeat (4) @(posedge clk);
    nrst = 1'b1;

    $display("--- tb_uart_baud_dds ---");

    // Test 1: divisor 0 disables both ticks on both instances.
    repeat (10_000) @(posedge clk);
    check(n_s16_std == 0 && n_bit_std == 0, "div 0: u_std silent");
    check(n_s16_fast == 0 && n_bit_fast == 0, "div 0: u_fast silent");
    check(n_s16_hs == 0 && n_bit_hs == 0 && n_s16_cb == 0 && n_bit_cb == 0,
          "div 0: u_hs/u_cb silent");

    // Test 2: exact average rates. 480,000 clocks at 48 MHz hold exactly 18,432
    // reference ticks (1.8432 MHz x 10 ms).
    rate_std(16'd1, 480_000, 18_432);  // 115200 baud x 16
    rate_std(16'd3, 480_000, 6_144);  // 38400
    rate_std(16'd12, 480_000, 1_536);  // 9600
    // 200 MHz: 1 ms = 200,000 clocks = 1,843.2 reference ticks. 3.579545 MHz:
    // 100 ms = 357,955 clocks (357,954.5 rounded) = 184,320 reference ticks.
    rate_x(16'd1, 200_000, 1_843, 357_955, 184_320);
    rate_x(16'd3, 200_000, 614, 357_955, 61_440);

    // Test 3: the whole 16-bit divisor range on u_fast: 65535 clocks per tick.
    @(negedge clk);
    div_fast = 16'hFFFF;
    period_fast(p);
    check(p == 65535, $sformatf("div 65535: %0d clocks per samp16 tick", p));
    @(negedge clk);
    div_fast = 16'd0;  // back through zero: every divisor bit falls
    repeat (8) @(posedge clk);

    // Test 4: divisor 2 -> a sample tick every 2 clocks, a bit tick every 32.
    @(negedge clk);
    div_fast = 16'd2;
    period_fast(p);
    check(p == 2, $sformatf("div 2: %0d clocks per samp16 tick", p));
    bit_period_fast(p);
    check(p == 32, $sformatf("div 2: %0d clocks per bit tick", p));

    // Test 5: resync restarts the bit phase. Ten clocks into a bit period
    // (five sample ticks in), pulse resync_i: the next bit tick must come
    // after exactly 16 more sample ticks, not the 11 the old phase owed.
    @(posedge bit_fast);
    repeat (10) @(posedge clk);
    @(negedge clk);
    resync_fast = 1'b1;
    @(negedge clk);
    resync_fast = 1'b0;
    // Count the sample ticks the divider takes from the resync edge on: the
    // one visible now is taken at the next clock, the first the restarted
    // phase counts.
    n           = 0;
    forever begin
      if (bit_fast) break;
      if (s16_fast) n++;
      @(negedge clk);
    end
    check(n == 16, $sformatf("resync: bit tick after %0d sample ticks (exp 16)", n));

    // Test 6: a divisor lowered below the running count fires at once.
    @(negedge clk);
    div_fast = 16'd100;
    @(posedge s16_fast);
    repeat (50) @(posedge clk);  // the count sits near 50
    @(negedge clk);
    div_fast = 16'd10;
    begin
      time t0;
      t0 = $time;
      @(posedge s16_fast);
      p = int'(($time - t0) / 10);
    end
    check(p <= 2, $sformatf("div 100->10 mid-count: tick after %0d clocks", p));
    period_fast(p);
    check(p == 10, $sformatf("div 10: %0d clocks per samp16 tick", p));

    // Test 7: reset mid-run silences both outputs and restarts the phase.
    @(negedge clk);
    div_std = 16'd1;
    repeat (100) @(posedge clk);
    @(negedge clk);
    nrst = 1'b0;
    @(negedge clk);
    check(s16_std === 1'b0 && bit_std === 1'b0 && s16_fast === 1'b0 && bit_fast === 1'b0,
          "reset: outputs low");
    s0 = n_s16_std;
    b0 = n_bit_fast;
    repeat (200) @(posedge clk);
    check(n_s16_std == s0 && n_bit_fast == b0, "reset: no ticks while held");
    @(negedge clk);
    nrst = 1'b1;
    period_fast(p);
    check(p == 10, $sformatf("after reset: %0d clocks per samp16 tick", p));
    rate_std(16'd1, 48_000, 1_843);

    // Disable at every phase of the /16 stage, including the clock when an
    // already-registered sample pulse would otherwise produce a bit pulse.
    // Observe pins only: u_fast/divisor 1 supplies one sample per core clock.
    for (int phase = 0; phase < 16; phase++) begin
      @(negedge clk);
      div_fast    = 16'd1;
      resync_fast = 1'b1;
      @(negedge clk);
      resync_fast = 1'b0;
      @(posedge bit_fast);
      repeat (phase) @(posedge clk);
      @(negedge clk);
      div_fast = 16'd0;
      repeat (4) begin
        @(negedge clk);
        check(s16_fast === 1'b0 && bit_fast === 1'b0, $sformatf(
              "disable at phase %0d: both registered outputs stay low", phase));
      end
    end

    if (errors == 0) $display("=== tb_uart_baud_dds: ALL CHECKS PASS ===");
    else $display("=== tb_uart_baud_dds: %0d FAILURE(S) ===", errors);
    if (errors != 0) $fatal(1, "tb_uart_baud_dds FAILED");
    $finish;
  end

  initial begin
    #50_000_000;
    $display("WATCHDOG timeout");
    $fatal(1, "tb_uart_baud_dds watchdog");
  end

endmodule

`default_nettype wire
