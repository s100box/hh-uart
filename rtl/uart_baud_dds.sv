// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// uart_baud_dds -- fractional, REF_HZ-relative UART baud generator.
//
//   CLK_HZ describes the actual input clock and REF_HZ <= CLK_HZ. The two-stage
//   divider avoids integer-divider rate error: changing a design's core clock
//   and matching CLK_HZ parameter preserves the firmware divisor's baud mapping.
//
//     Stage 1 (DDS): a phase accumulator synthesizes the REF_HZ crystal rate
//       from clk_i -- average rate exact, <= one clk_i of jitter.
//     Stage 2: divide the REF_HZ tick by `divisor_i` to get the 16x oversample
//       tick (samp16_tick_o), then by 16 to get the per-bit tick (bit_tick_o).
//       baud = REF_HZ / (16 * divisor_i)  (divisor 1 -> 115200 at REF_HZ
//       1.8432 MHz; 3 -> 38400; ...). `divisor_i` is a runtime input so a UART
//       with a programmable rate (a 16550's DLL/DLM) drives it live; a fixed-
//       rate UART ties it to a constant. divisor_i == 0 suppresses both output
//       ticks on the next clock edge and holds the partial /16 phase. resync_i
//       can clear that phase even while disabled.
//
//   `resync_i` (pulse) restarts the /16 sample-token count; the next bit tick
//   follows sixteen subsequently consumed sample ticks. The phase to the first
//   sample remains free-running, so this is not a full bit-period delay from
//   the resync edge. A consumer starting TX off-tick must account for that
//   phase. uart_tx_core starts and advances only on bit_tick_o; its consumer
//   ties the generator's resync_i to 0. samp16_tick_o remains free-running for RX.
//
//   One generator serves TX on bit_tick_o and RX on samp16_tick_o; their
//   fixed one-clock pipeline offset does not change the steady-state rates.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_baud_dds #(
    parameter int unsigned CLK_HZ = 48_000_000,  // core clock
    parameter int unsigned REF_HZ = 1_843_200    // UART reference crystal
) (
    input  wire logic        clk_i,
    input  wire logic        nrst_i,         // synchronous, active-low
    input  wire logic [15:0] divisor_i,      // baud = REF_HZ / (16 * divisor_i)
    input  wire logic        resync_i,       // pulse: restart the /16 sample-token count
    output logic             samp16_tick_o,  // 16x-per-bit RX sample tick (free-running)
    output logic             bit_tick_o      // one tick per bit period
);

  // ── Stage 1 DDS: exact REF_HZ from clk_i ────────────────────────────────────
  logic [31:0] ref_acc_q;
  logic        ref_tick;
  wire  [32:0] ref_sum = {1'b0, ref_acc_q} + REF_HZ;
  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      ref_acc_q <= 32'd0;
      ref_tick  <= 1'b0;
    end else if (ref_sum >= 33'(CLK_HZ)) begin
      // The remainder fits 32 bits even when ref_sum carries into bit 32;
      // modulo-2^32 subtraction of its low word preserves that remainder.
      ref_acc_q <= ref_sum[31:0] - CLK_HZ;
      ref_tick  <= 1'b1;
    end else begin
      ref_acc_q <= ref_sum[31:0];
      ref_tick  <= 1'b0;
    end
  end

  // ── Stage 2a: REF_HZ / divisor -> 16x oversample tick ──────────────────────
  logic [15:0] samp_cnt_q;
  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      samp_cnt_q    <= 16'd0;
      samp16_tick_o <= 1'b0;
    end else begin
      samp16_tick_o <= 1'b0;
      if (divisor_i == 16'd0) begin
        samp_cnt_q <= 16'd0;  // divisor 0 = disabled
      end else if (ref_tick) begin
        if (samp_cnt_q + 16'd1 >= divisor_i) begin
          samp_cnt_q    <= 16'd0;
          samp16_tick_o <= 1'b1;
        end else begin
          samp_cnt_q <= samp_cnt_q + 16'd1;
        end
      end
    end
  end

  // ── Stage 2b: /16 -> per-bit tick (re-phaseable on resync_i) ───────────────
  logic [3:0] bit_cnt_q;
  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      bit_cnt_q  <= 4'd0;
      bit_tick_o <= 1'b0;
    end else begin
      bit_tick_o <= 1'b0;
      if (resync_i) begin
        bit_cnt_q <= 4'd0;
      end else if (divisor_i != 16'd0 && samp16_tick_o) begin
        // The registered sample pulse belongs to the preceding clock. Do not
        // consume it after disable, or a final bit tick can escape at phase 15.
        // A disabled divisor holds the partial bit phase; resync still clears it.
        if (bit_cnt_q == 4'd15) begin
          bit_cnt_q  <= 4'd0;
          bit_tick_o <= 1'b1;
        end else begin
          bit_cnt_q <= bit_cnt_q + 4'd1;
        end
      end
    end
  end

endmodule : uart_baud_dds

`default_nettype wire
