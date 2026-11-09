// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// usb_serial_fifo -- byte FIFO for the host-UART <-> USB bridge channel.
//
//   One module, two builds selected by SAME_CLOCK:
//     SAME_CLOCK=0 (default): a true asynchronous FIFO -- binary pointers
//       converted to Gray, each side's pointer crossed into the other clock
//       domain through a 2-FF synchronizer, full/empty computed locally. Use
//       when the host core and the USB host stack run on different clocks.
//       Constrain delay/skew from rd_gray_q->rd_gray_m and wr_gray_q->wr_gray_m
//       so successive Gray changes cannot overlap. A false path alone does
//       not bound skew; keep the synchronizer's second stage normally timed.
//       See doc/hh-uart-design.md section 6.2 for the integration contract.
//     SAME_CLOCK=1: a plain single-clock FIFO (binary pointers, no Gray code, no
//       synchronizers) -- lower latency, fewer LUTs, zero CDC. rd_clk_i /
//       rd_nrst_i are then unused (tie to the write-side clock/reset).
//
//   DEPTH must be a power of two: at least 4 in the asynchronous build, or
//   2 in the single-clock build. Async read data (rdata_o = mem[rd]) lets a
//   downstream consumer register it; `re_i` advances the read pointer.
//============================================================================
`default_nettype none

module usb_serial_fifo #(
    parameter int unsigned DW         = 8,
    parameter int unsigned DEPTH      = 16,   // power of two
    parameter bit          SAME_CLOCK = 1'b0
) (
    // write domain
    input  wire logic          wr_clk_i,
    input  wire logic          wr_nrst_i,
    input  wire logic          we_i,
    input  wire logic [DW-1:0] wdata_i,
    output logic               full_o,
    // read domain (== write domain when SAME_CLOCK)
    input  wire logic          rd_clk_i,
    input  wire logic          rd_nrst_i,
    input  wire logic          re_i,
    output logic      [DW-1:0] rdata_o,
    output logic               empty_o
);

  localparam int unsigned AW = $clog2(DEPTH);
  localparam int unsigned PW = AW + 1;  // extra MSB disambiguates full vs empty

  logic [DW-1:0] mem[DEPTH];

  generate
    if (SAME_CLOCK) begin : g_sync
      // ---- Single-clock FIFO (binary pointers) ----------------------------
      logic [PW-1:0] wr_bin_q, rd_bin_q;
      wire wr_en = we_i & ~full_o;
      wire rd_en = re_i & ~empty_o;

      assign full_o  = (wr_bin_q[PW-1] != rd_bin_q[PW-1]) && (wr_bin_q[AW-1:0] == rd_bin_q[AW-1:0]);
      assign empty_o = (wr_bin_q == rd_bin_q);
      assign rdata_o = mem[rd_bin_q[AW-1:0]];

      always_ff @(posedge wr_clk_i) begin
        if (!wr_nrst_i) begin
          wr_bin_q <= '0;
          rd_bin_q <= '0;
        end else begin
          if (wr_en) begin
            mem[wr_bin_q[AW-1:0]] <= wdata_i;
            wr_bin_q              <= wr_bin_q + 1'b1;
          end
          if (rd_en) rd_bin_q <= rd_bin_q + 1'b1;
        end
      end
    end else begin : g_async
      // ---- Asynchronous FIFO (Gray pointers + 2-FF synchronizers) ---------
      // Cummings-style: full/empty are REGISTERED so the pointer-increment
      // enables depend only on the registered flag (no combinational loop), and
      // each gray pointer crosses to the other domain through a 2-FF chain.
      logic [PW-1:0] wr_bin_q, wr_gray_q, rd_bin_q, rd_gray_q;
      logic [PW-1:0] rd_gray_m, rd_gray_s;  // rd ptr in the write domain
      logic [PW-1:0] wr_gray_m, wr_gray_s;  // wr ptr in the read domain
      logic full_q, empty_q;

      wire wr_en = we_i & ~full_q;
      wire rd_en = re_i & ~empty_q;
      wire [PW-1:0] wr_bin_nxt = wr_bin_q + (wr_en ? PW'(1) : PW'(0));
      wire [PW-1:0] rd_bin_nxt = rd_bin_q + (rd_en ? PW'(1) : PW'(0));
      wire [PW-1:0] wr_gray_nxt = wr_bin_nxt ^ (wr_bin_nxt >> 1);
      wire [PW-1:0] rd_gray_nxt = rd_bin_nxt ^ (rd_bin_nxt >> 1);
      // Full when the next write gray equals the synced read gray with the top
      // two bits inverted (the classic async-FIFO test); empty when the next
      // read gray catches the synced write gray.
      wire full_nxt = (wr_gray_nxt == {~rd_gray_s[PW-1:PW-2], rd_gray_s[PW-3:0]});
      wire empty_nxt = (rd_gray_nxt == wr_gray_s);

      assign rdata_o = mem[rd_bin_q[AW-1:0]];
      assign full_o  = full_q;
      assign empty_o = empty_q;

      // Write domain.
      always_ff @(posedge wr_clk_i) begin
        if (!wr_nrst_i) begin
          wr_bin_q  <= '0;
          wr_gray_q <= '0;
          rd_gray_m <= '0;
          rd_gray_s <= '0;
          full_q    <= 1'b0;
        end else begin
          if (wr_en) mem[wr_bin_q[AW-1:0]] <= wdata_i;
          wr_bin_q  <= wr_bin_nxt;
          wr_gray_q <= wr_gray_nxt;
          rd_gray_m <= rd_gray_q;  // CDC: read-pointer gray into write domain
          rd_gray_s <= rd_gray_m;
          full_q    <= full_nxt;
        end
      end

      // Read domain.
      always_ff @(posedge rd_clk_i) begin
        if (!rd_nrst_i) begin
          rd_bin_q  <= '0;
          rd_gray_q <= '0;
          wr_gray_m <= '0;
          wr_gray_s <= '0;
          empty_q   <= 1'b1;
        end else begin
          rd_bin_q  <= rd_bin_nxt;
          rd_gray_q <= rd_gray_nxt;
          wr_gray_m <= wr_gray_q;  // CDC: write-pointer gray into read domain
          wr_gray_s <= wr_gray_m;
          empty_q   <= empty_nxt;
        end
      end
    end
  endgenerate

endmodule

`default_nettype wire
