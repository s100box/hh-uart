// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// uart_rx_core -- configurable-frame serial receiver (16x oversampling).
//
//   One deserializer for any register-compatible UART (16550, Z180 ASCI, 6551,
//   Z80 DART and the like): 5-8 data bits, none/even/odd/mark/space parity,
//   and break detection, all runtime inputs. A UART with fixed framing ties the
//   config inputs to constants (8N1 = word_len 3, parity_en 0).
//
//   samp16_tick_i is a clock enable at 16 times the line rate. The receiver
//   confirms a start after eight sample ticks, then samples each data/parity/
//   stop position sixteen ticks later. Fractional core-clocks-per-bit ratios
//   therefore use the same sample grid as integer ratios, without estimating
//   a bit duration in core-clock cycles. Keep framing inputs stable for the
//   complete received frame; they are used live rather than latched at start.
//
//   Receive flow: idle HIGH; start bit (1->0); confirm at mid-start (rejects a
//   glitch); sample n data bits LSB-first at mid-bit; optional parity bit; stop
//   bit. A 2-FF synchronizer guards the pin. Re-aligns to every start bit, so
//   any baud the parent's samp16 follows is tracked. On a complete frame it
//   pulses stb_o with the byte on data_o and the error flags on err_o:
//     err_o[0] = PE  parity error   (sampled parity != expected; only when PEN)
//     err_o[1] = FE  framing error  (stop bit sampled LOW)
//     err_o[2] = BI  break          (the whole frame -- data + parity + stop --
//                                     sampled spacing/low)
//   The errored character is still delivered (the byte is received, flag set).
//   A UART that wants only {PE,FE} takes err_o[1:0]; one that wants none leaves
//   err_o unconnected. Synchronous active-low reset.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_rx_core (
    input wire logic clk_i,
    input wire logic nrst_i,         // synchronous, active-low
    input wire logic samp16_tick_i,  // 16x-per-bit oversample tick (parent baud gen)
    input wire logic rxd_i,          // serial line in (idle high)

    // ---- Frame config ----
    input  wire logic [1:0] word_len_i,      // 0=5, 1=6, 2=7, 3=8 data bits
    input  wire logic       parity_en_i,     // a parity bit is present
    input  wire logic       parity_even_i,   // 1=even, 0=odd (when !stick)
    input  wire logic       parity_stick_i,  // parity is a fixed value
    output logic      [7:0] data_o,          // received byte (valid with stb_o)
    output logic      [2:0] err_o,           // {BI, FE, PE} for data_o
    // The RAW bit received in the parity slot, valid with stb_o. Normally this
    // is redundant with err_o[0] (PE says whether it matched), but a protocol
    // can put DATA in that slot instead of a check -- the Z180 ASCI's
    // multiprocessor mode is one: UM005 frames it as "Start bit + 7 or 8 data
    // bits + MPB bit + 1 or 2 stop bits" and says the format "has no provision
    // for parity", so the bit is the ninth data bit and PE is meaningless. Such
    // a caller drives parity_en_i from its own mode bit, reads the bit here,
    // and ignores PE. Leave it unconnected for ordinary parity, where it says
    // nothing err_o does not.
    output logic            parity_bit_o,
    output logic            stb_o            // one-cycle strobe: data_o is fresh
);

  // ---- 2-FF synchronizer for the asynchronous RXD pin --------------------------
  // rxd_sync_d1 delays rxd_sync one clock so idle detects a start as a FALLING
  // edge, not a level: a line that is merely still low (a framing-error stop tail,
  // or a break) cannot masquerade as a fresh start bit.
  logic rxd_meta;
  logic rxd_sync;
  logic rxd_sync_d1;
  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      rxd_meta    <= 1'b1;
      rxd_sync    <= 1'b1;
      rxd_sync_d1 <= 1'b1;
    end else begin
      rxd_meta    <= rxd_i;
      rxd_sync    <= rxd_meta;
      rxd_sync_d1 <= rxd_sync;
    end
  end
  wire rxd_fall = rxd_sync_d1 && !rxd_sync;  // start bit = idle-high -> low edge

  // ---- Receiver FSM (16x oversampling; one sample tick advances it) ------------
  localparam logic [2:0] StIdle = 3'd0;
  localparam logic [2:0] StStart = 3'd1;
  localparam logic [2:0] StData = 3'd2;
  localparam logic [2:0] StParity = 3'd3;
  localparam logic [2:0] StStop = 3'd4;

  // Sample the middle of a 16-tick bit window. After the start-bit edge we wait
  // 8 ticks to mid-start (SampMid), then a full 16 ticks (SampFull) to reach the
  // mid-point of every subsequent bit.
  localparam logic [3:0] SampMid = 4'd7;  // 8th tick (0..7) = mid of start bit
  localparam logic [3:0] SampFull = 4'd15;  // 16th tick (0..15) = mid of next bit

  logic [2:0] state_q;
  logic [3:0] samp_cnt_q;  // 16x sample ticks elapsed within the current bit
  logic [2:0] bit_idx_q;  // data bit index 0..7
  logic [7:0] rx_sr_q;  // accumulates data, LSB-first, into bit positions
  logic [7:0] data_q;
  logic [2:0] err_q;
  logic       pe_q;  // parity error for the frame in flight
  logic       pbit_q;  // the raw bit sampled in the parity slot
  logic       pbit_hold_q;  // that bit, held with data_o
  logic       zero_q;  // all data + parity bits seen so far were 0 (break track)
  logic       stb_q;

  logic [3:0] ndata;
  assign ndata = 4'd5 + {2'b00, word_len_i};

  // Expected parity bit over the recovered data byte (masked to ndata):
  //   even -> XOR(data); odd -> ~XOR(data); stick -> fixed (EPS=1 space/0, EPS=0 mark/1).
  logic [7:0] data_mask;
  logic       data_xor;
  logic       exp_parity;
  assign data_mask  = 8'hFF >> (8 - ndata);
  assign data_xor   = ^(rx_sr_q & data_mask);
  assign exp_parity = parity_stick_i ? ~parity_even_i : (parity_even_i ? data_xor : ~data_xor);

  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      state_q     <= StIdle;
      samp_cnt_q  <= 4'd0;
      bit_idx_q   <= 3'd0;
      rx_sr_q     <= 8'h00;
      data_q      <= 8'h00;
      err_q       <= 3'b000;
      pe_q        <= 1'b0;
      pbit_q      <= 1'b0;
      pbit_hold_q <= 1'b0;
      zero_q      <= 1'b1;
      stb_q       <= 1'b0;
    end else begin
      stb_q <= 1'b0;  // one-cycle strobe by default
      case (state_q)
        // Idle (mark). A start bit is a falling edge on the line; begin the count.
        StIdle: begin
          samp_cnt_q <= 4'd0;
          bit_idx_q  <= 3'd0;
          pe_q       <= 1'b0;
          // Cleared per frame like pe_q, so a frame with no parity slot
          // presents 0 here rather than the previous frame's bit.
          pbit_q     <= 1'b0;
          rx_sr_q    <= 8'h00;
          zero_q     <= 1'b1;
          if (rxd_fall) state_q <= StStart;
        end

        // Confirm the start bit at its mid-point (rejects a glitch), then align:
        // the next sample is a full bit period (16 ticks) later = mid of data 0.
        StStart: begin
          if (samp16_tick_i) begin
            if (samp_cnt_q == SampMid) begin
              samp_cnt_q <= 4'd0;
              if (rxd_sync == 1'b0) state_q <= StData;  // real start
              else state_q <= StIdle;  // glitch
            end else begin
              samp_cnt_q <= samp_cnt_q + 4'd1;
            end
          end
        end

        // Sample each data bit mid-bit (16 ticks after the previous sample),
        // LSB first, into its bit position so a short word right-aligns.
        StData: begin
          if (samp16_tick_i) begin
            if (samp_cnt_q == SampFull) begin
              samp_cnt_q         <= 4'd0;
              rx_sr_q[bit_idx_q] <= rxd_sync;
              if (rxd_sync) zero_q <= 1'b0;
              if (bit_idx_q == ndata[2:0] - 3'd1) state_q <= parity_en_i ? StParity : StStop;
              else bit_idx_q <= bit_idx_q + 3'd1;
            end else begin
              samp_cnt_q <= samp_cnt_q + 4'd1;
            end
          end
        end

        // Parity bit (only when PEN): sample mid-bit, flag a mismatch vs expected.
        StParity: begin
          if (samp16_tick_i) begin
            if (samp_cnt_q == SampFull) begin
              samp_cnt_q <= 4'd0;
              pe_q       <= (rxd_sync != exp_parity);
              pbit_q     <= rxd_sync;
              if (rxd_sync) zero_q <= 1'b0;
              state_q <= StStop;
            end else begin
              samp_cnt_q <= samp_cnt_q + 4'd1;
            end
          end
        end

        // Stop bit: present the byte + errors at mid-stop and strobe. FE = stop
        // sampled LOW; BI = the entire frame (data + parity + stop) was spacing.
        StStop: begin
          if (samp16_tick_i) begin
            if (samp_cnt_q == SampFull) begin
              data_q      <= rx_sr_q;
              err_q       <= {(zero_q && (rxd_sync == 1'b0)), (rxd_sync != 1'b1), pe_q};
              pbit_hold_q <= pbit_q;
              stb_q       <= 1'b1;
              state_q     <= StIdle;
            end else begin
              samp_cnt_q <= samp_cnt_q + 4'd1;
            end
          end
        end

        default: state_q <= StIdle;
      endcase
    end
  end

  assign data_o       = data_q;
  assign err_o        = err_q;
  assign parity_bit_o = pbit_hold_q;
  assign stb_o        = stb_q;

endmodule : uart_rx_core

`default_nettype wire
