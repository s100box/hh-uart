// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Native Yosys driver: one time unit per global step. The model still allows
// independent write/read pauses; the coordinator itself cannot stutter.
module usb_fifo_clock_formal (
    input wire       advance_wr,
    advance_rd,
    restart,
    we,
    re,
    input wire [7:0] data
);
  reg formal_clk = 0;
  always @($global_clock) formal_clk <= !formal_clk;
  usb_fifo_formal model (
      .formal_clk(formal_clk),
      .advance_wr(advance_wr),
      .advance_rd(advance_rd),
      .restart   (restart),
      .we        (we),
      .re        (re),
      .data      (data)
  );
endmodule
