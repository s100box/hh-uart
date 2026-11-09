// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// Formal properties bound into the three UART cores (SymbiYosys, prove mode).
//
//   These are the invariants doc/coverage.md relies on to classify the points
//   no stimulus can reach: the state registers never hold a code outside the
//   named states (so each core's `default` arm is unreachable), the data
//   masks' low five bits are constant 1 (a frame has at least five data bits),
//   and the DDS accumulator stays below CLK_HZ (so its bits above
//   log2(CLK_HZ) are constant 0). The transmitter is proved in both builds,
//   with and without its holding buffer: a byte taken goes to the buffer, or
//   (HOLD = 0) starts its frame at once, and without the buffer ready_o is
//   high only on a baud tick. Each is proved by k-induction from the reset
//   state; `bind` keeps the RTL itself untouched. The formal run is
//   `ninja formal` in sim/.
//============================================================================
`default_nettype none

module uart_rx_core_props (
    input wire logic       clk_i,
    input wire logic       nrst_i,
    input wire logic [2:0] state_q,
    input wire logic [7:0] data_mask
);
  logic past_valid = 1'b0;
  always_ff @(posedge clk_i) past_valid <= 1'b1;
  always_comb if (!past_valid) assume (!nrst_i);  // start in reset
  always_ff @(posedge clk_i) begin
    if (past_valid) begin
      assert (state_q <= 3'd4);  // StIdle..StStop only: the default arm is dead
      assert (data_mask[4:0] == 5'h1F);
    end
  end
endmodule

module uart_tx_core_props #(
    parameter bit HOLD = 1'b1
) (
    input wire logic       clk_i,
    input wire logic       nrst_i,
    input wire logic [2:0] state_q,
    input wire logic [7:0] dmask_load,
    input wire logic       baud_tick_i,
    input wire logic       we_i,
    input wire logic       ready_o,
    input wire logic       hold_valid
);
  logic past_valid = 1'b0;
  logic took_q = 1'b0;  // the last clock took a byte (we_i && ready_o, out of reset)
  always_ff @(posedge clk_i) begin
    past_valid <= 1'b1;
    took_q     <= nrst_i && we_i && ready_o;
  end
  always_comb if (!past_valid) assume (!nrst_i);
  always_ff @(posedge clk_i) begin
    if (past_valid) begin
      assert (state_q == 3'd0 || state_q == 3'd2 || state_q == 3'd3 || state_q == 3'd4);
      assert (dmask_load[4:0] == 5'h1F);
      if (HOLD) begin
        if (took_q && nrst_i) assert (hold_valid);
      end else begin
        assert (!hold_valid);
        assert (!ready_o || baud_tick_i);
        if (took_q && nrst_i) assert (state_q == 3'd2);  // StData: its start bit is out
      end
    end
  end
endmodule

module uart_baud_dds_props #(
    parameter int unsigned CLK_HZ = 200_000_000
) (
    input wire logic        clk_i,
    input wire logic        nrst_i,
    input wire logic [31:0] ref_acc_q,
    input wire logic [32:0] ref_sum,
    input wire logic [15:0] divisor_i,
    input wire logic        resync_i,
    input wire logic        samp16_tick_o,
    input wire logic        bit_tick_o,
    input wire logic [ 3:0] bit_cnt_q
);
  logic past_valid = 1'b0;
  always_ff @(posedge clk_i) past_valid <= 1'b1;
  always_comb if (!past_valid) assume (!nrst_i);
  always_ff @(posedge clk_i) begin
    if (past_valid) begin
      assert (ref_acc_q < CLK_HZ);  // the inductive invariant
      assert (ref_acc_q[31:28] == 4'd0);  // what it implies for the toggle holes
      assert (ref_sum[32:28] == 5'd0);
      if (!$past(nrst_i) || $past(divisor_i) == 16'd0) assert (!samp16_tick_o && !bit_tick_o);
      if ($past(nrst_i) && $past(divisor_i) == 16'd0 && !$past(resync_i))
        assert (bit_cnt_q == $past(bit_cnt_q));
    end
  end
endmodule

bind uart_rx_core uart_rx_core_props p_rx (
    .clk_i    (clk_i),
    .nrst_i   (nrst_i),
    .state_q  (state_q),
    .data_mask(data_mask)
);
bind uart_tx_core uart_tx_core_props #(
    .HOLD(HOLD)
) p_tx (
    .clk_i      (clk_i),
    .nrst_i     (nrst_i),
    .state_q    (state_q),
    .dmask_load (dmask_load),
    .baud_tick_i(baud_tick_i),
    .we_i       (we_i),
    .ready_o    (ready_o),
    .hold_valid (hold_valid)
);
bind uart_baud_dds uart_baud_dds_props #(
    .CLK_HZ(CLK_HZ)
) p_baud (
    .clk_i        (clk_i),
    .nrst_i       (nrst_i),
    .ref_acc_q    (ref_acc_q),
    .ref_sum      (ref_sum),
    .divisor_i    (divisor_i),
    .resync_i     (resync_i),
    .samp16_tick_o(samp16_tick_o),
    .bit_tick_o   (bit_tick_o),
    .bit_cnt_q    (bit_cnt_q)
);

// One top so a single sby task proves all three cores (the transmitter in both
// builds); the generator takes the fastest clock the benches use, where the
// accumulator spans the most bits.
module uart_cores_formal (
    input wire logic        clk_i,
    input wire logic        nrst_i,
    input wire logic        samp16_tick_i,
    input wire logic        rxd_i,
    input wire logic [ 1:0] word_len_i,
    input wire logic        parity_en_i,
    input wire logic        parity_even_i,
    input wire logic        parity_stick_i,
    input wire logic [ 7:0] data_i,
    input wire logic        we_i,
    input wire logic        stop2_i,
    input wire logic        break_i,
    input wire logic [15:0] divisor_i,
    input wire logic        resync_i
);
  logic [7:0] rx_data;
  logic [2:0] rx_err;
  logic rx_pbit, rx_stb, tx_ready, tx_idle, txd, s16, bt;
  logic tx0_ready, tx0_idle, txd0;

  uart_rx_core u_rx (
      .clk_i         (clk_i),
      .nrst_i        (nrst_i),
      .samp16_tick_i (samp16_tick_i),
      .rxd_i         (rxd_i),
      .word_len_i    (word_len_i),
      .parity_en_i   (parity_en_i),
      .parity_even_i (parity_even_i),
      .parity_stick_i(parity_stick_i),
      .data_o        (rx_data),
      .err_o         (rx_err),
      .parity_bit_o  (rx_pbit),
      .stb_o         (rx_stb)
  );

  uart_tx_core u_tx (
      .clk_i         (clk_i),
      .nrst_i        (nrst_i),
      .baud_tick_i   (samp16_tick_i),
      .data_i        (data_i),
      .we_i          (we_i),
      .word_len_i    (word_len_i),
      .parity_en_i   (parity_en_i),
      .parity_even_i (parity_even_i),
      .parity_stick_i(parity_stick_i),
      .stop2_i       (stop2_i),
      .break_i       (break_i),
      .ready_o       (tx_ready),
      .tx_idle_o     (tx_idle),
      .txd_o         (txd)
  );

  uart_tx_core #(
      .HOLD(1'b0)
  ) u_tx0 (
      .clk_i         (clk_i),
      .nrst_i        (nrst_i),
      .baud_tick_i   (samp16_tick_i),
      .data_i        (data_i),
      .we_i          (we_i),
      .word_len_i    (word_len_i),
      .parity_en_i   (parity_en_i),
      .parity_even_i (parity_even_i),
      .parity_stick_i(parity_stick_i),
      .stop2_i       (stop2_i),
      .break_i       (break_i),
      .ready_o       (tx0_ready),
      .tx_idle_o     (tx0_idle),
      .txd_o         (txd0)
  );

  uart_baud_dds #(
      .CLK_HZ(200_000_000),
      .REF_HZ(1_843_200)
  ) u_baud (
      .clk_i        (clk_i),
      .nrst_i       (nrst_i),
      .divisor_i    (divisor_i),
      .resync_i     (resync_i),
      .samp16_tick_o(s16),
      .bit_tick_o   (bt)
  );
endmodule

`default_nettype wire
