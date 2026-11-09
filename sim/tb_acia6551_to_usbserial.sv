// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_acia6551_to_usbserial -- the 6551 ACIA adapter on a shared-clock
// usb_serial_chan, driven from a model of the ACIA's two FIFOs on one side and
// the stack's register window on the other:
//   * TX: one byte; then a 20-byte burst that fills the 16-byte channel FIFO,
//     with tx_consumed dropping exactly at full and the order kept end to end.
//   * RX: a byte held off while rx_full, written when room appears; a
//     4-byte run in order.
//   * Sidebands: {RTS, DTR} read back asserted, BAUD_PENDING never set.
//============================================================================
`default_nettype none

module tb_acia6551_to_usbserial;

  logic clk = 1'b0, nrst = 1'b0;
  always #5 clk = ~clk;

  integer errors = 0;
  task automatic check(input logic c, input string s);
    if (c !== 1'b1) begin
      $display("FAIL: %s", s);
      errors = errors + 1;
    end
  endtask

  localparam logic [2:0] RegStatus = 3'd0, RegDataTx = 3'd1, RegDataRx = 3'd2, RegMctl = 3'd7;
  localparam int Depth = 16;

  usb_serial_if us ();
  logic [2:0] v_addr;
  logic v_we, v_re;
  logic [7:0] v_wdata, v_rdata;

  logic [7:0] tx_byte;
  logic       tx_valid;
  logic       tx_consumed;
  logic [7:0] rx_byte;
  logic       rx_strobe;
  logic       rx_full;

  acia6551_to_usbserial dut (
      .tx_byte_i    (tx_byte),
      .tx_valid_i   (tx_valid),
      .tx_consumed_o(tx_consumed),
      .rx_byte_o    (rx_byte),
      .rx_strobe_o  (rx_strobe),
      .rx_full_i    (rx_full),
      .us           (us)
  );

  usb_serial_chan #(
      .DEPTH     (Depth),
      .SAME_CLOCK(1'b1)
  ) u_chan (
      .host_clk_i (clk),
      .host_nrst_i(nrst),
      .us         (us),
      .v_clk_i    (clk),
      .v_nrst_i   (nrst),
      .v_addr_i   (v_addr),
      .v_we_i     (v_we),
      .v_re_i     (v_re),
      .v_wdata_i  (v_wdata),
      .v_rdata_o  (v_rdata)
  );

  // ---- the ACIA's TX FIFO: head at tx_byte, popped on tx_consumed ---------
  logic [7:0] acia_tx_mem[64];
  integer tx_wr = 0, tx_rd = 0, consumed_count = 0;
  assign tx_valid = (tx_wr != tx_rd);
  assign tx_byte  = tx_valid ? acia_tx_mem[tx_rd] : 8'h00;
  task automatic acia_tx_push(input logic [7:0] b);
    acia_tx_mem[tx_wr] = b;
    tx_wr              = tx_wr + 1;
  endtask
  always @(posedge clk) begin
    if (tx_consumed) begin
      check(tx_valid, "tx_consumed only with tx_valid");
      tx_rd          <= tx_rd + 1;
      consumed_count <= consumed_count + 1;
    end
  end

  // ---- the ACIA's RX FIFO write port: record every strobe ------------------
  logic [7:0] acia_rx_got[$];
  always @(posedge clk) if (rx_strobe) acia_rx_got.push_back(rx_byte);

  task automatic v_read(input logic [2:0] a, output logic [7:0] d);
    @(negedge clk);
    v_addr = a;
    v_re   = 1'b1;
    #1 d = v_rdata;
    @(posedge clk);
    @(negedge clk);
    v_re = 1'b0;
  endtask
  task automatic v_write(input logic [2:0] a, input logic [7:0] d);
    @(negedge clk);
    v_addr  = a;
    v_wdata = d;
    v_we    = 1'b1;
    @(posedge clk);
    @(negedge clk);
    v_we = 1'b0;
  endtask

  logic [7:0] tmp;

  initial begin
    rx_full = 1'b0;
    v_addr  = 0;
    v_we    = 0;
    v_re    = 0;
    v_wdata = 0;
    repeat (6) @(posedge clk);
    nrst = 1;
    repeat (6) @(posedge clk);

    // ===== TX: one byte, consumed in the cycle the channel takes it =====
    @(negedge clk);
    acia_tx_push(8'hFF);  // exercise every data bit, then return to idle zero
    #1 check(tx_consumed === 1'b1, "TX: tx_consumed with room in the channel");
    @(posedge clk);
    #1 check(tx_wr == tx_rd && tx_consumed === 1'b0, "TX: one pop, then idle");
    v_read(RegStatus, tmp);
    check(tmp[0] === 1'b1, "TX: TX_AVAIL after the byte");
    v_read(RegDataTx, tmp);
    check(tmp === 8'hFF, "TX: the stack reads the byte");
    v_read(RegStatus, tmp);
    check(tmp[0] === 1'b0, "TX: TX_AVAIL clear after the read");

    // ===== TX burst: 20 bytes into a 16-byte channel FIFO =====
    consumed_count = 0;
    @(negedge clk);
    for (int i = 0; i < 20; i++) acia_tx_push(8'(8'h40 + i));
    repeat (Depth + 8) @(posedge clk);
    #1;
    check(consumed_count == Depth, $sformatf(
          "TX burst: %0d accepted, want %0d", consumed_count, Depth));
    check(tx_wr - tx_rd == 20 - Depth && tx_valid === 1'b1 && tx_consumed === 1'b0,
          "TX burst: the rest held in the ACIA while the channel is full");
    for (int i = 0; i < 20; i++) begin
      v_read(RegDataTx, tmp);
      check(tmp === 8'(8'h40 + i), $sformatf(
            "TX burst: byte %0d = %02h, want %02h", i, tmp, 8'(8'h40 + i)));
      repeat (2) @(posedge clk);
    end
    check(tx_wr == tx_rd, "TX burst: the ACIA drained");

    // ===== RX: held off while full, written when room appears =====
    @(negedge clk);
    rx_full = 1'b1;
    v_write(RegDataRx, 8'hFF);  // every RX data bit must cross the adapter
    repeat (3) @(posedge clk);
    check(acia_rx_got.size() == 0, "RX: nothing written while rx_full");
    check(us.rx_valid === 1'b1, "RX: the byte waits in the channel");
    @(negedge clk);
    rx_full = 1'b0;
    #1 check(rx_strobe === 1'b1 && rx_byte === 8'hFF, "RX: written when room appears");
    @(posedge clk);
    #1 check(rx_strobe === 1'b0, "RX: one strobe per byte");
    for (int i = 0; i < 4; i++) v_write(RegDataRx, 8'(8'hA0 + i));
    repeat (4) @(posedge clk);
    check(acia_rx_got.size() == 5, $sformatf("RX: %0d bytes written, want 5", acia_rx_got.size()));
    for (int i = 0; i < 5; i++)
    check(acia_rx_got[i] === (i == 0 ? 8'hFF : 8'(8'hA0 + i - 1)), $sformatf(
          "RX: byte %0d = %02h", i, acia_rx_got[i]));

    // ===== Sidebands =====
    v_read(RegMctl, tmp);
    check(tmp[1:0] === 2'b11, "MCTL: {RTS, DTR} asserted");
    v_read(RegStatus, tmp);
    check(tmp[2] === 1'b0, "STATUS: BAUD_PENDING never set");

    if (errors == 0) $display("=== tb_acia6551_to_usbserial: ALL CHECKS PASS ===");
    else $display("=== tb_acia6551_to_usbserial: %0d FAILURE(S) ===", errors);
    if (errors != 0) $fatal(1, "tb_acia6551_to_usbserial FAILED");
    $finish;
  end

  initial begin
    #500_000;
    $display("WATCHDOG");
    $fatal(1, "tb_acia6551_to_usbserial watchdog");
  end

endmodule

`default_nettype wire
