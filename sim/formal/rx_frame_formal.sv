// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// A sample-age model, independent of the receiver's state encodings. The line
// and tick may change arbitrarily. Only frame configuration must remain stable
// while receiving, as required of the parent UART by the public interface.
module rx_frame_formal (
    input wire logic       clk,
    input wire logic       nrst,
    input wire logic       tick,
    input wire logic       line_in,
    input wire logic [1:0] length,
    input wire logic       parity_enable,
    input wire logic       parity_even,
    input wire logic       parity_stick
);
  wire [7:0] data_out;
  wire [2:0] errors;
  wire parity_out, strobe;
  uart_rx_core dut (
      .clk_i         (clk),
      .nrst_i        (nrst),
      .samp16_tick_i (tick),
      .rxd_i         (line_in),
      .word_len_i    (length),
      .parity_en_i   (parity_enable),
      .parity_even_i (parity_even),
      .parity_stick_i(parity_stick),
      .data_o        (data_out),
      .err_o         (errors),
      .parity_bit_o  (parity_out),
      .stb_o         (strobe)
  );
  logic past_valid = 0;
  always @(posedge clk) past_valid <= 1;
  always_comb if (!past_valid) assume (!nrst);
  logic [2:0] line_pipe;
  logic active;
  logic [7:0] age, payload, expected_data;
  logic parity_slot, parity_error, expected_parity, expected_strobe;
  logic [2:0] expected_errors;
  wire [3:0] data_bits = 5 + {2'b0, length};
  wire [3:0] slot = age[7:4];
  wire sample = tick && age[3:0] == 7;
  wire [3:0] stop_slot = data_bits + 1'b1 + 4'(parity_enable);
  logic correct_parity;
  always_comb begin
    correct_parity = !parity_even;
    if (!parity_stick) for (int i = 0; i < 8; i++) if (i < data_bits) correct_parity ^= payload[i];
  end
  always @(posedge clk) begin
    if (!nrst) begin
      line_pipe       <= 3'b111;
      active          <= 0;
      age             <= 0;
      payload         <= 0;
      parity_slot     <= 0;
      parity_error    <= 0;
      expected_data   <= 0;
      expected_parity <= 0;
      expected_errors <= 0;
      expected_strobe <= 0;
    end else begin
      line_pipe       <= {line_pipe[1:0], line_in};
      expected_strobe <= 0;
      if (!active) begin
        age          <= 0;
        payload      <= 0;
        parity_slot  <= 0;
        parity_error <= 0;
        if (line_pipe[2] && !line_pipe[1]) active <= 1;
      end else if (tick) begin
        age <= age + 1'b1;
        if (sample) begin
          if (slot == 0) begin
            if (line_pipe[1]) active <= 0;
          end else if (slot <= data_bits) payload[slot-1'b1] <= line_pipe[1];
          else if (slot < stop_slot) begin
            parity_slot  <= line_pipe[1];
            parity_error <= line_pipe[1] != correct_parity;
          end else begin
            active <= 0;
            expected_strobe <= 1;
            expected_data <= payload;
            expected_parity <= parity_slot;
            expected_errors <= {
              !(|payload) && !parity_slot && !line_pipe[1], !line_pipe[1], parity_error
            };
          end
        end
      end
    end
    if (past_valid) begin
      if (nrst && $past(nrst) && $past(active))
        assume ({length, parity_enable, parity_even, parity_stick} == $past(
            {length, parity_enable, parity_even, parity_stick}
        ));
      assert (data_out == expected_data);
      assert (errors == expected_errors);
      assert (parity_out == expected_parity);
      assert (strobe == expected_strobe);
      assert ({dut.rxd_sync_d1, dut.rxd_sync, dut.rxd_meta} == line_pipe);
      if (active && nrst) begin
        assert (dut.rx_sr_q == payload);
        assert (dut.pbit_q == parity_slot);
        assert (dut.pe_q == parity_error);
        assert (dut.zero_q == (!(|payload) && !parity_slot));
        if (age < 8) begin
          assert (dut.state_q == 1);
          assert (dut.samp_cnt_q == age[3:0]);
        end else begin
          assert (dut.samp_cnt_q == age[3:0] - 4'd8);
          if (age < 8 + 16 * data_bits) begin
            assert (dut.state_q == 2);
            assert (dut.bit_idx_q == (age - 8) / 16);
          end else if (parity_enable && age < 8 + 16 * (data_bits + 1))
            assert (dut.state_q == 3);
            else assert (dut.state_q == 4);
        end
      end else if (!active) assert (dut.state_q == 0);
      cover (strobe && data_out == 8'hA5 && errors == 0);
      cover (strobe && errors == 3'b110);
      cover (strobe && errors[0] && length == 0 && parity_enable);
      cover ($past(active && age == 7 && tick && line_pipe[1]) && !active && !strobe);
`ifdef FORMAL_NEGATIVE
      assert (!strobe);
`endif
    end
  end
endmodule
