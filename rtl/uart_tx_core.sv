// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// uart_tx_core -- configurable-frame serial transmitter.
//
//   One serializer for any register-compatible UART (16550, Z180 ASCI, 6551,
//   Z80 DART and the like), carrying the full 16550 frame: 5-8 data bits,
//   none/even/odd/mark/space parity, 1 or 2 stop bits, and break (force the
//   line to spacing). A UART with fixed framing ties the config inputs to
//   constants (8N1 = word_len 3, parity_en 0, stop2 0).
//
//   The producer is throttled to the line rate by baud_tick_i (one pulse per
//   bit period, from the parent's baud generator) and the ready_o handshake.
//   HOLD = 1 (default) puts a 1-deep holding buffer in front of the shifter;
//   HOLD = 0 has none, for a parent that keeps its waiting bytes itself:
//   ready_o is then high only in the clock the shifter can load, and a byte
//   offered with we_i in that clock starts its frame on that tick (a we_i in
//   any other clock is ignored). The bit FSM advances on baud_tick_i (start ->
//   n data LSB-first -> optional parity -> 1/2 stop), so the wire bit is just
//   the current state's output. The frame's data-bit count and parity are
//   latched when the byte loads. stop2_i is sampled at the last data bit;
//   hold it stable until then, or change framing only while tx_idle_o is high.
//   Synchronous active-low reset.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_tx_core #(
    // 1 = a 1-deep holding buffer before the shifter; 0 = none (see above).
    parameter bit HOLD = 1'b1
) (
    input wire logic clk_i,
    input wire logic nrst_i,      // synchronous, active-low
    input wire logic baud_tick_i, // one pulse per bit period (parent baud gen)

    input  wire logic [7:0] data_i,          // byte to transmit
    input  wire logic       we_i,            // one-cycle enqueue strobe (data_i valid)
    // ---- Frame config (length/parity at load; stop2_i at the final data bit) ----
    input  wire logic [1:0] word_len_i,      // 0=5, 1=6, 2=7, 3=8 data bits
    input  wire logic       parity_en_i,     // add a parity bit
    input  wire logic       parity_even_i,   // 1=even, 0=odd (when !stick)
    input  wire logic       parity_stick_i,  // force parity to a fixed value
    input  wire logic       stop2_i,         // 1 = 2 stop bits (else 1)
    input  wire logic       break_i,         // force TXD spacing (low)
    output logic            ready_o,         // we_i will be taken this clock
    output logic            tx_idle_o,       // shifter empty AND holding buffer (if any) empty
    output logic            txd_o            // serial line out (idle high)
);

  localparam logic [2:0] StIdle = 3'd0;
  localparam logic [2:0] StData = 3'd2;
  localparam logic [2:0] StParity = 3'd3;
  localparam logic [2:0] StStop = 3'd4;

  logic [2:0] state_q;
  logic [2:0] bit_idx_q;  // current data bit being shifted (0-based)
  logic [1:0] stop_rem_q;  // stop-bit periods remaining in the active frame
  logic [7:0] sr_q;  // data byte in flight
  logic [3:0] ndata_q;  // data bit count of the frame in flight (5..8)
  logic par_q;  // parity bit value for the frame in flight
  logic paren_q;  // this frame has a parity bit
  logic txd_q;  // registered line bit (held between baud ticks)

  logic [7:0] hold;  // 1-deep holding buffer (HOLD = 1; constant 0 without it)
  logic hold_valid;

  // The shifter can load on this tick: idle, or the frame's last stop bit done.
  wire slot = baud_tick_i && (state_q == StIdle || (state_q == StStop && stop_rem_q == 2'd0));
  // A byte to load, and which: the holding buffer's, or the one offered now.
  wire have = HOLD ? hold_valid : we_i;
  wire [7:0] next_byte = HOLD ? hold : data_i;

  assign ready_o   = HOLD ? ~hold_valid : slot;
  assign txd_o     = break_i ? 1'b0 : txd_q;
  assign tx_idle_o = (state_q == StIdle) && !hold_valid;

  // Parity for the byte to load (computed combinationally; latched at load).
  wire [3:0] ndata_load = 4'd5 + {2'b00, word_len_i};
  wire [7:0] dmask_load = 8'hFF >> (8 - ndata_load);
  wire       dxor_load = ^(next_byte & dmask_load);
  wire       par_load = parity_stick_i ? ~parity_even_i : (parity_even_i ? dxor_load : ~dxor_load);

  if (HOLD) begin : g_hold
    // we_i fills the buffer when it is free (a same-clock load cannot happen,
    // since the buffer was empty); a frame starting from it empties it.
    always_ff @(posedge clk_i) begin
      if (!nrst_i) begin
        hold       <= 8'h00;
        hold_valid <= 1'b0;
      end else if (we_i && !hold_valid) begin
        hold       <= data_i;
        hold_valid <= 1'b1;
      end else if (slot && hold_valid) begin
        hold_valid <= 1'b0;
      end
    end
  end else begin : g_direct
    assign hold       = 8'h00;
    assign hold_valid = 1'b0;
  end

  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      state_q    <= StIdle;
      bit_idx_q  <= 3'd0;
      stop_rem_q <= 2'd0;
      sr_q       <= 8'h00;
      ndata_q    <= 4'd8;
      par_q      <= 1'b0;
      paren_q    <= 1'b0;
      txd_q      <= 1'b1;  // idle (mark)
    end else begin
      if (baud_tick_i) begin
        case (state_q)
          StIdle: begin
            txd_q <= 1'b1;
            if (have) begin
              // Begin a frame: emit the start bit, latch the byte + framing.
              txd_q     <= 1'b0;
              sr_q      <= next_byte;
              ndata_q   <= ndata_load;
              par_q     <= par_load;
              paren_q   <= parity_en_i;
              bit_idx_q <= 3'd0;
              state_q   <= StData;
            end
          end

          StData: begin
            txd_q <= sr_q[bit_idx_q];
            if (bit_idx_q == ndata_q[2:0] - 3'd1) begin
              stop_rem_q <= stop2_i ? 2'd2 : 2'd1;
              state_q    <= paren_q ? StParity : StStop;
            end else begin
              bit_idx_q <= bit_idx_q + 3'd1;
            end
          end

          StParity: begin
            txd_q   <= par_q;
            state_q <= StStop;
          end

          // Hold the line marking for the frame's stop bit period(s). Only once
          // they are fully sent do we begin the next byte (or go idle), so each
          // stop bit occupies its own full bit period (no back-to-back squash).
          StStop: begin
            if (stop_rem_q != 2'd0) begin
              txd_q      <= 1'b1;  // stop bit
              stop_rem_q <= stop_rem_q - 2'd1;
            end else if (have) begin
              txd_q     <= 1'b0;  // start bit of the next byte, back to back
              sr_q      <= next_byte;
              ndata_q   <= ndata_load;
              par_q     <= par_load;
              paren_q   <= parity_en_i;
              bit_idx_q <= 3'd0;
              state_q   <= StData;
            end else begin
              txd_q   <= 1'b1;
              state_q <= StIdle;
            end
          end

          default: state_q <= StIdle;
        endcase
      end
    end
  end

endmodule : uart_tx_core

`default_nettype wire
