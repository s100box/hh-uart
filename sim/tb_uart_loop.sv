// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_uart_loop -- the three UART cores wired as a UART, transmitter looped
// back into receiver, on one uart_baud_dds.
//
//   The generator runs at a fractional clocks-per-bit ratio (40.5 MHz /
//   115200 = 351.5625), the case the 16x-grid receiver exists for: the
//   transmitter advances on bit_tick_o, the receiver samples on samp16_tick_o,
//   and every byte pushed in must come out intact with no error flag, for
//   every word length, parity mode and stop-bit count, at three divisors.
//============================================================================
`default_nettype none

module tb_uart_loop;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic nrst = 1'b0;

  logic [15:0] divisor;
  logic samp16, bit_tick;
  uart_baud_dds #(
      .CLK_HZ(40_500_000),
      .REF_HZ(1_843_200)
  ) u_baud (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (divisor),
      .resync_i     (1'b0),
      .samp16_tick_o(samp16),
      .bit_tick_o   (bit_tick)
  );

  logic [1:0] word_len;
  logic parity_en, parity_even, parity_stick, stop2;
  logic [7:0] tx_data;
  logic tx_we, tx_ready, tx_idle, txd;
  uart_tx_core u_tx (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .baud_tick_i   (bit_tick),
      .data_i        (tx_data),
      .we_i          (tx_we),
      .word_len_i    (word_len),
      .parity_en_i   (parity_en),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .stop2_i       (stop2),
      .break_i       (1'b0),
      .ready_o       (tx_ready),
      .tx_idle_o     (tx_idle),
      .txd_o         (txd)
  );

  logic [7:0] rx_data;
  logic [2:0] rx_err;
  logic rx_pbit, rx_stb;
  uart_rx_core u_rx (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .samp16_tick_i (samp16),
      .rxd_i         (txd),
      .word_len_i    (word_len),
      .parity_en_i   (parity_en),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .data_o        (rx_data),
      .err_o         (rx_err),
      .parity_bit_o  (rx_pbit),
      .stb_o         (rx_stb)
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

  task automatic set_frame(input int wl, input int par, input logic s2);
    begin
      word_len     = wl[1:0];
      parity_en    = (par != PNone);
      parity_stick = (par == PMark) || (par == PSpace);
      parity_even  = (par == PEven) || (par == PSpace);
      stop2        = s2;
    end
  endtask

  function automatic logic [7:0] masked(input logic [7:0] b, input int wl);
    return b & (8'hFF >> (8 - (5 + wl)));
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

  // Receiver side: collect every delivery.
  localparam int NBytes = 6;
  logic [7:0] got[NBytes];
  logic [2:0] got_err[NBytes];
  logic got_pb[NBytes];
  int got_n = 0;
  always @(posedge clk) begin
    if (rx_stb) begin
      if (got_n < NBytes) begin
        got[got_n]     <= rx_data;
        got_err[got_n] <= rx_err;
        got_pb[got_n]  <= rx_pbit;
      end
      got_n <= got_n + 1;
    end
  end

  logic [7:0] sent[NBytes];

  // Push NBytes through the transmitter as fast as ready_o allows, wait for
  // the receiver to deliver them all, and compare.
  task automatic run_block(input int wl, input int par, input logic s2, input logic [15:0] d);
    string tag;
    int waited;
    begin
      tag = $sformatf("div %0d %0d%s%0d", d, 5 + wl, pname(par), s2 ? 2 : 1);
      set_frame(wl, par, s2);
      @(negedge clk);
      divisor = d;
      repeat (200) @(posedge clk);
      sent[0] = 8'h00;
      sent[1] = 8'hFF;
      sent[2] = 8'h55;
      sent[3] = 8'hAA;
      sent[4] = 8'($urandom);
      sent[5] = 8'($urandom);
      got_n   = 0;
      for (int i = 0; i < NBytes; i++) begin
        @(negedge clk);
        while (!tx_ready) @(negedge clk);
        tx_data = sent[i];
        tx_we   = 1'b1;
        @(negedge clk);
        tx_we = 1'b0;
      end
      // Wait for every byte, bounded by 3x the block's wire time.
      waited = 0;
      while (got_n < NBytes && waited < 3 * NBytes * 12 * 16 * 22 * d) begin
        @(posedge clk);
        waited++;
      end
      check(got_n == NBytes, $sformatf("%s: %0d of %0d bytes delivered", tag, got_n, NBytes));
      for (int i = 0; i < NBytes && i < got_n; i++) begin
        check(got[i] === masked(sent[i], wl), $sformatf(
              "%s: byte %0d (sent %02h got %02h)", tag, i, masked(sent[i], wl), got[i]));
        check(got_err[i] === 3'b000, $sformatf("%s: byte %0d err %b", tag, i, got_err[i]));
        check(got_pb[i] === ((par != PNone) ? exp_parity(sent[i], wl, par) : 1'b0), $sformatf(
              "%s: byte %0d parity slot %b", tag, i, got_pb[i]));
      end
      // The transmitter must have drained too.
      repeat (16 * 22 * d * 2) @(posedge clk);
      check(tx_idle === 1'b1, {tag, ": tx idle after the block"});
    end
  endtask

  initial begin
    divisor = 16'd0;
    tx_data = 8'h00;
    tx_we   = 1'b0;
    set_frame(3, PNone, 1'b0);
    nrst = 1'b0;
    repeat (4) @(posedge clk);
    nrst = 1'b1;

    $display("--- tb_uart_loop ---");

    // Divisor 1 (115200 at 40.5 MHz: 351.5625 clocks per bit): the full matrix.
    for (int wl = 0; wl < 4; wl++)
    for (int par = PNone; par <= PSpace; par++)
    for (int s2 = 0; s2 < 2; s2++) run_block(wl, par, s2[0], 16'd1);

    // Slower rates: a sample of configs.
    run_block(3, PNone, 1'b0, 16'd3);
    run_block(2, PEven, 1'b1, 16'd3);
    run_block(3, POdd, 1'b0, 16'd12);

    if (errors == 0) $display("=== tb_uart_loop: ALL CHECKS PASS ===");
    else $display("=== tb_uart_loop: %0d FAILURE(S) ===", errors);
    if (errors != 0) $fatal(1, "tb_uart_loop FAILED");
    $finish;
  end

  initial begin
    #200_000_000;
    $display("WATCHDOG timeout");
    $fatal(1, "tb_uart_loop watchdog");
  end

endmodule

`default_nettype wire
