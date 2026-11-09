// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Independent bit-position model: start, 5-8 data bits, optional parity and
// 1/2 stops. Configuration may change at any clock; stop2 is sampled at the
// final data bit, matching the public transmitter contract.
module tx_frame_formal #(
    parameter bit HOLD = 0
) (
    input wire logic       clk,
    input wire logic       nrst,
    input wire logic       tick,
    input wire logic       we,
    input wire logic [7:0] data,
    input wire logic [1:0] length,
    input wire logic       parity_enable,
    input wire logic       parity_even,
    input wire logic       parity_stick,
    input wire logic       two_stops,
    input wire logic       break_line
);
  wire ready, txd;
  uart_tx_core #(
      .HOLD(HOLD)
  ) dut (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .baud_tick_i   (tick),
      .data_i        (data),
      .we_i          (we),
      .word_len_i    (length),
      .parity_en_i   (parity_enable),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .stop2_i       (two_stops),
      .break_i       (break_line),
      .ready_o       (ready),
      .tx_idle_o     (),
      .txd_o         (txd)
  );
  logic past_valid = 0;
  always_ff @(posedge clk) past_valid <= 1;
  always_comb if (!past_valid) assume (!nrst);
  logic queued;
  logic [7:0] queued_byte, frame_byte;
  logic active, has_parity, parity_bit, second_stop, expected;
  logic [3:0] position, data_bits;
  wire [4:0] last_position = {1'b0, data_bits} + 5'(has_parity) + (second_stop ? 5'd2 : 5'd1);
  wire available = !active || 5'(position) == last_position;
  wire start = tick && available && (HOLD ? queued : we);
  wire [7:0] next_byte = HOLD ? queued_byte : data;
  logic next_parity;
  always_comb begin
    next_parity = !parity_even;
    if (!parity_stick)
      for (int i = 0; i < 8; i++) if (i < 5 + int'(length)) next_parity ^= next_byte[i];
  end
  always_ff @(posedge clk) begin
    if (!nrst) begin
      queued      <= 0;
      active      <= 0;
      expected    <= 1;
      position    <= 0;
      data_bits   <= 8;
      has_parity  <= 0;
      parity_bit  <= 0;
      second_stop <= 0;
    end else begin
      if (HOLD) begin
        if (we && !queued) begin
          queued      <= 1;
          queued_byte <= data;
        end else if (start) queued <= 0;
      end
      if (start) begin
        active      <= 1;
        position    <= 0;
        frame_byte  <= next_byte;
        data_bits   <= 5 + {2'b0, length};
        has_parity  <= parity_enable;
        parity_bit  <= next_parity;
        second_stop <= 0;
        expected    <= 0;
      end else if (tick) begin
        if (available) begin
          active   <= 0;
          expected <= 1;
        end else begin
          position <= position + 1'b1;
          if (position < data_bits) begin
            expected <= frame_byte[position];
            if (position + 1'b1 == data_bits) second_stop <= two_stops;
          end else if (has_parity && position == data_bits) expected <= parity_bit;
          else expected <= 1;
        end
      end
    end
    if (past_valid) begin
      assert (txd == (break_line ? 1'b0 : expected));
      assert (ready == (HOLD ? !queued : tick && available));
      if (HOLD) begin
        assert (dut.hold_valid == queued);
        if (queued) assert (dut.hold == queued_byte);
      end
      if (active) begin
        assert (dut.sr_q == frame_byte);
        assert (dut.ndata_q == data_bits);
        assert (dut.paren_q == has_parity);
        assert (dut.par_q == parity_bit);
      end
      cover (active && available && frame_byte == 8'hA5);
      cover (active && available && data_bits == 5 && has_parity && second_stop);
`ifdef FORMAL_NEGATIVE
      if (active && position == 0 && !break_line) assert (txd);
`endif
    end
  end
endmodule
