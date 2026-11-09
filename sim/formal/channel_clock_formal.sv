// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Native global timebase; the two domain clocks remain independently pausable.
module channel_clock_formal (
    input wire        advance_host,
    advance_stack,
    input wire [ 7:0] tx_byte,
    stack_write_data,
    input wire        tx_valid,
    rx_full,
    input wire [ 2:0] address,
    input wire        stack_write,
    stack_read,
    input wire [31:0] baud_divisor,
    input wire        baud_strobe,
    input wire [ 1:0] modem_out
);
  reg formal_clk = 0;
  always @($global_clock) formal_clk <= !formal_clk;
  channel_formal model (
      .formal_clk      (formal_clk),
      .advance_host    (advance_host),
      .advance_stack   (advance_stack),
      .tx_byte         (tx_byte),
      .stack_write_data(stack_write_data),
      .tx_valid        (tx_valid),
      .rx_full         (rx_full),
      .address         (address),
      .stack_write     (stack_write),
      .stack_read      (stack_read),
      .baud_divisor    (baud_divisor),
      .baud_strobe     (baud_strobe),
      .modem_out       (modem_out)
  );
endmodule
