// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// Formal properties bound into uart16550 (SymbiYosys, prove mode).
//
//   The FIFO bookkeeping the design relies on, proved by k-induction from
//   reset with every CPU, serial and modem input free:
//     * each FIFO's tail is its head plus its count, and each count is at
//       most 16;
//     * the RX FIFO's errored-entry count equals the entries in the FIFO that
//       carry an error, and FIFOE is set whenever it is nonzero and clear in
//       16450 mode;
//     * the transmitter (no holding buffer) is offered a byte only in the
//       clock it can load, and always then while a byte waits and no clear
//       discards it; the TX FIFO never overflows; the 16450 THR is unused in
//       FIFO mode and the TX FIFO is empty in 16450 mode;
//     * the THRE interrupt is pending, or held back, only while THRE is set.
//   `bind` keeps the RTL itself untouched. The formal run is `ninja formal`.
//============================================================================
`default_nettype none

module uart16550_props (
    input wire logic        clk_i,
    input wire logic        nrst_i,
    input wire logic        fifo_active,
    input wire logic [ 4:0] rxf_count_q,
    input wire logic [ 3:0] rxf_head_q,
    input wire logic [ 3:0] rxf_tail_q,
    input wire logic [47:0] rxf_err,       // {entry 15, ..., entry 0}, 3 bits each
    input wire logic [ 4:0] rxf_errcnt_q,
    input wire logic        fifoe_q,
    input wire logic [ 4:0] txf_count_q,
    input wire logic [ 3:0] txf_head_q,
    input wire logic [ 3:0] txf_tail_q,
    input wire logic        tx_push,
    input wire logic        tx_pop,
    input wire logic        tx_clr,
    input wire logic        tx_we,
    input wire logic        tx_ready,
    input wire logic        thr_valid_q,
    input wire logic        tx_empty,
    input wire logic        thre_pend_q,
    input wire logic        thre_dly_q
);
  // Entries from the head that carry an error.
  function automatic logic [4:0] errored(input logic [47:0] e, input logic [3:0] head,
                                         input logic [4:0] count);
    logic [3:0] slot;
    errored = 5'd0;
    for (int k = 0; k < 16; k++) begin
      slot = head + 4'(k);
      if (5'(k) < count && e[3*slot+:3] != 3'd0) errored = errored + 5'd1;
    end
  endfunction

  logic past_valid = 1'b0;
  always_ff @(posedge clk_i) past_valid <= 1'b1;
  always_comb if (!past_valid) assume (!nrst_i);  // start in reset
  always_ff @(posedge clk_i) begin
    if (past_valid) begin
      assert (rxf_count_q <= 5'd16);
      assert (rxf_tail_q == rxf_head_q + rxf_count_q[3:0]);
      assert (txf_count_q <= 5'd16);
      assert (txf_tail_q == txf_head_q + txf_count_q[3:0]);
      assert (rxf_errcnt_q == errored(rxf_err, rxf_head_q, rxf_count_q));
      assert (rxf_errcnt_q == 5'd0 || fifoe_q);
      assert (fifo_active || !fifoe_q);
      assert (!tx_we || tx_ready);
      assert (!(tx_ready && !tx_empty && !tx_clr) || tx_we);
      assert (!(tx_push && !tx_pop && !tx_clr && txf_count_q == 5'd16));
      assert (!fifo_active || !thr_valid_q);
      assert (fifo_active || txf_count_q == 5'd0);
      assert (!thre_pend_q || tx_empty);
      assert (!thre_dly_q || (tx_empty && fifo_active));
    end
  end
endmodule

bind uart16550 uart16550_props p_uart (
    .clk_i(clk_i),
    .nrst_i(nrst_i),
    .fifo_active(fifo_active),
    .rxf_count_q(rxf_count_q),
    .rxf_head_q(rxf_head_q),
    .rxf_tail_q(rxf_tail_q),
    .rxf_err({
      rxf_err_q[15],
      rxf_err_q[14],
      rxf_err_q[13],
      rxf_err_q[12],
      rxf_err_q[11],
      rxf_err_q[10],
      rxf_err_q[9],
      rxf_err_q[8],
      rxf_err_q[7],
      rxf_err_q[6],
      rxf_err_q[5],
      rxf_err_q[4],
      rxf_err_q[3],
      rxf_err_q[2],
      rxf_err_q[1],
      rxf_err_q[0]
    }),
    .rxf_errcnt_q(rxf_errcnt_q),
    .fifoe_q(fifoe_q),
    .txf_count_q(txf_count_q),
    .txf_head_q(txf_head_q),
    .txf_tail_q(txf_tail_q),
    .tx_push(tx_push),
    .tx_pop(tx_pop),
    .tx_clr(tx_clr),
    .tx_we(tx_we),
    .tx_ready(tx_ready),
    .thr_valid_q(thr_valid_q),
    .tx_empty(tx_empty),
    .thre_pend_q(thre_pend_q),
    .thre_dly_q(thre_dly_q)
);

// The 16550A with every input free. CLK_HZ = 4 x REF_HZ keeps the baud
// generator small; the properties hold at any divisor.
module uart16550_formal #(
    parameter int VARIANT = 3,
    parameter int CLK_HZ  = 7_372_800
) (
    input  wire logic       clk_i,
    input  wire logic       nrst_i,
    input  wire logic       cs_i,
    input  wire logic [2:0] addr_i,
    input  wire logic       rd_i,
    input  wire logic       wr_i,
    input  wire logic [7:0] din_i,
    input  wire logic       rxd_i,
    input  wire logic       cts_n_i,
    input  wire logic       dsr_n_i,
    input  wire logic       dcd_n_i,
    input  wire logic       ri_n_i,
    output logic      [7:0] dout_o,
    output logic            int_o
);
  logic [7:0] vec;
  logic txd, rts_n, dtr_n, bt, s16;

  uart16550 #(
      .VARIANT(VARIANT),
      .CLK_HZ (CLK_HZ),
      .REF_HZ (1_843_200)
  ) dut (
      .clk_i             (clk_i),
      .nrst_i            (nrst_i),
      .cs_i              (cs_i),
      .addr_i            (addr_i),
      .rd_i              (rd_i),
      .wr_i              (wr_i),
      .din_i             (din_i),
      .dout_o            (dout_o),
      .int_o             (int_o),
      .int_vec_o         (vec),
      .txd_o             (txd),
      .rxd_i             (rxd_i),
      .cts_n_i           (cts_n_i),
      .dsr_n_i           (dsr_n_i),
      .dcd_n_i           (dcd_n_i),
      .ri_n_i            (ri_n_i),
      .rts_n_o           (rts_n),
      .dtr_n_o           (dtr_n),
      .baud_bit_tick_o   (bt),
      .baud_samp16_tick_o(s16)
  );

  // Bus-visible register preservation, DLAB banking, and interrupt priority.
  // Inputs remain unconstrained after the initial reset, including simultaneous
  // CPU accesses and serial/modem changes. These properties add no fairness.
  logic bus_past_valid = 0;
  logic [3:0] next_modem;
  wire modem_armed = dut.mcr_loop || dut.msr_arm_q == 3;
  wire [2:0] fresh_line_errors =
      ((dut.fifo_active && dut.rxf_count_q != 0 && !dut.rxf_shown_q) ?
       dut.rxf_err_q[dut.rxf_head_q] : 3'b000) |
      ((!dut.fifo_active && dut.rx_stb) ? dut.rx_err : 3'b000);
  always_comb begin
    next_modem = dut.mcr_loop ?
        {dut.mcr_out2, dut.mcr_out1, dut.mcr_dtr, dut.mcr_rts} :
        {dut.dcd_s, dut.ri_s, dut.dsr_s, dut.cts_s};
  end
  always_ff @(posedge clk_i) begin
    bus_past_valid <= 1;
    if (bus_past_valid && nrst_i && $past(nrst_i)) begin
      assert (dut.dll_q == ($past(dut.wr_dll) ? $past(din_i) : $past(dut.dll_q)));
      assert (dut.dlm_q == ($past(dut.wr_dlm) ? $past(din_i) : $past(dut.dlm_q)));
      assert (dut.lcr_q == ($past(dut.wr_lcr) ? $past(din_i) : $past(dut.lcr_q)));
      assert (dut.mcr_q == ($past(dut.wr_mcr) ? $past(din_i[4:0]) : $past(dut.mcr_q)));
      assert (dut.ier_q == ($past(dut.wr_ier) ? $past(din_i[3:0]) : $past(dut.ier_q)));
      assert (dut.rbr_q == ($past(
          !dut.fifo_active && dut.rx_stb
      ) ? $past(
          dut.rx_byte
      ) : $past(
          dut.rbr_q
      )));
      assert (dut.rbr_valid_q == ($past(
          dut.fcr_mode_chg
      ) ? 1'b0 : $past(
          !dut.fifo_active && dut.rx_stb
      ) ? 1'b1 : $past(
          !dut.fifo_active && dut.rd_rbr
      ) ? 1'b0 : $past(
          dut.rbr_valid_q
      )));
      assert (dut.thr_q == ($past(dut.thr_load) ? $past(din_i) : $past(dut.thr_q)));
      assert (dut.thr_valid_q == ($past(
          dut.fcr_mode_chg
      ) ? 1'b0 : $past(
          dut.thr_load
      ) ? 1'b1 : $past(
          dut.thr_move
      ) ? 1'b0 : $past(
          dut.thr_valid_q
      )));
      if (VARIANT != 0)
        assert (dut.scr_q == ($past(dut.wr_scr) ? $past(din_i) : $past(dut.scr_q)));
        else assert (dut.scr_q == 0);
      // A newly exposed FIFO head reasserts its errors in the read cycle.
      assert (dut.lsr_oe_q == ($past(
          (dut.fifo_active && dut.rx_stb && !dut.rx_push) ||
                 (!dut.fifo_active && dut.rx_stb && dut.rbr_valid_q && !dut.rd_rbr)
      ) || $past(
          dut.lsr_oe_q && !dut.rd_lsr
      )));
      assert (dut.lsr_pe_q == ($past(fresh_line_errors[0]) || $past(dut.lsr_pe_q && !dut.rd_lsr)));
      assert (dut.lsr_fe_q == ($past(fresh_line_errors[1]) || $past(dut.lsr_fe_q && !dut.rd_lsr)));
      assert (dut.lsr_bi_q == ($past(fresh_line_errors[2]) || $past(dut.lsr_bi_q && !dut.rd_lsr)));
      if ($past(
              dut.rd_lsr
          ) && !$past(
              dut.rx_stb
          ) && !$past(
              dut.fifo_active && dut.rxf_count_q != 0 && !dut.rxf_shown_q
          )) begin
        assert (!dut.lsr_oe_q && !dut.lsr_pe_q && !dut.lsr_fe_q && !dut.lsr_bi_q);
      end
      // A new delta wins over an MSR read in the same cycle. TERI is the
      // falling edge of asserted RI; the other three deltas detect either edge.
      assert ({dut.msr_dcd_q, dut.msr_ri_q, dut.msr_dsr_q, dut.msr_cts_q} == $past(next_modem));
      assert (dut.msr_dcts_q == ($past(
          modem_armed && (next_modem[0] != dut.msr_cts_q)
      ) || $past(
          !dut.rd_msr && dut.msr_dcts_q
      )));
      assert (dut.msr_ddsr_q == ($past(
          modem_armed && (next_modem[1] != dut.msr_dsr_q)
      ) || $past(
          !dut.rd_msr && dut.msr_ddsr_q
      )));
      assert (dut.msr_teri_q == ($past(
          modem_armed && dut.msr_ri_q && !next_modem[2]
      ) || $past(
          !dut.rd_msr && dut.msr_teri_q
      )));
      assert (dut.msr_ddcd_q == ($past(
          modem_armed && (next_modem[3] != dut.msr_dcd_q)
      ) || $past(
          !dut.rd_msr && dut.msr_ddcd_q
      )));
      assert ({dut.cts_m, dut.dsr_m, dut.ri_m, dut.dcd_m} == ~$past(
          {cts_n_i, dsr_n_i, ri_n_i, dcd_n_i}
      ));
      if ($past(nrst_i, 2)) begin
        assert ({dut.cts_s, dut.dsr_s, dut.ri_s, dut.dcd_s} ==
                ~$past({cts_n_i, dsr_n_i, ri_n_i, dcd_n_i}, 2));
      end
      // RX activity/read/clear restarts the inactivity measurement. The
      // timeout is raised on the tick observing an already-reached threshold.
      if ($past(dut.rx_stb || dut.rd_rbr || dut.rx_clr)) begin
        assert (dut.to_cnt_q == 0 && !dut.to_fired_q);
      end else if ($past(dut.fifo_active && dut.rxf_count_q != 0 && dut.baud_tick)) begin
        if ($past(dut.to_cnt_q >= dut.to_threshold)) begin
          assert (dut.to_fired_q);
          assert (dut.to_cnt_q == $past(dut.to_cnt_q));
        end else begin
          assert (dut.to_cnt_q == $past(dut.to_cnt_q) + 6'd1);
          assert (dut.to_fired_q == $past(dut.to_fired_q));
        end
      end else begin
        assert (dut.to_cnt_q == $past(dut.to_cnt_q));
        assert (dut.to_fired_q == $past(dut.to_fired_q));
      end
      // THRE acknowledgement, write and clear priorities are bus-cycle exact.
      if ($past(dut.tx_clr))
        assert (dut.thre_pend_q && !dut.thre_dly_q);
        else if ($past(dut.wr_rbr_thr))
          assert (!dut.thre_pend_q && !dut.thre_dly_q);
          else if ($past(dut.rd_iir && dut.iir_code == 2))
            assert (!dut.thre_pend_q);
            else if ($past(
                    (dut.wr_ier && din_i[1] && dut.tx_empty) ||
                    (dut.thre_dly_q && dut.baud_tick && dut.thre_dly_cnt_q == 1) ||
                    (dut.thre_rise && (!dut.fifo_active || dut.thre_two_q || dut.thre_fcr_q))
                ))
              assert (dut.thre_pend_q);
              else assert (dut.thre_pend_q == $past(dut.thre_pend_q));
      if (!$past(dut.tx_clr || dut.wr_rbr_thr || (dut.wr_ier && din_i[1] && dut.tx_empty))) begin
        if ($past(dut.thre_dly_q && dut.baud_tick)) begin
          if ($past(dut.thre_dly_cnt_q == 1))
            assert (!dut.thre_dly_q);
            else begin
              assert (dut.thre_dly_q);
              assert (dut.thre_dly_cnt_q == $past(dut.thre_dly_cnt_q) - 4'd1);
            end
        end else if ($past(
                dut.thre_rise && dut.fifo_active && !dut.thre_two_q && !dut.thre_fcr_q
            )) begin
          assert (dut.thre_dly_q);
          assert (dut.thre_dly_cnt_q == $past(dut.frame_bits) - 4'd1);
        end else begin
          assert (dut.thre_dly_q == $past(dut.thre_dly_q));
          assert (dut.thre_dly_cnt_q == $past(dut.thre_dly_cnt_q));
        end
      end
`ifdef FORMAL_NEGATIVE
      assert (dut.dll_q == $past(dut.dll_q));  // false on a DLAB/DLL write
`endif
    end
    if (bus_past_valid && nrst_i) begin
      if (dut.ier_q[2] && dut.line_status)
        assert (dut.iir_code == 4'h6);
        else if (dut.ier_q[0] && dut.rda_active)
          assert (dut.iir_code == 4'h4);
          else if (dut.ier_q[0] && dut.to_active)
            assert (dut.iir_code == 4'hC);
            else if (dut.ier_q[1] && dut.thre_pend_q)
              assert (dut.iir_code == 4'h2);
              else if (dut.ier_q[3] && dut.modem_delta)
                assert (dut.iir_code == 4'h0);
                else assert (dut.iir_code == 4'h1);
      assert (int_o == ((dut.mcr_q[4] || dut.mcr_q[3]) && !dut.iir_code[0]));
      assert (vec == 8'hFF);
      assert (rts_n == !(dut.mcr_rts && !dut.mcr_loop));
      assert (dtr_n == !(dut.mcr_dtr && !dut.mcr_loop));
      if (dut.rd_rbr) assert (dout_o == dut.rbr_head);
      if (dut.rd_lsr)
        assert (dout_o == {dut.fifoe_q, dut.tx_empty && dut.tx_idle, dut.tx_empty,
                          dut.lsr_bi_q, dut.lsr_fe_q, dut.lsr_pe_q, dut.lsr_oe_q,
                          dut.fifo_active ? dut.rxf_count_q != 0 : dut.rbr_valid_q});
      if (dut.rd_msr)
        assert (dout_o == {dut.mcr_loop ? {dut.mcr_out2, dut.mcr_out1, dut.mcr_dtr, dut.mcr_rts} :
                          {dut.msr_dcd_q, dut.msr_ri_q, dut.msr_dsr_q, dut.msr_cts_q},
                          dut.msr_ddcd_q, dut.msr_teri_q, dut.msr_ddsr_q, dut.msr_dcts_q});
      cover (dut.rd_iir && dut.iir_code == 4'h2 && int_o);
      cover (dut.wr_dll && din_i == 8'hA5);
      cover (dut.rd_msr && dut.msr_teri_q && modem_armed && !next_modem[2]);
    end
  end
endmodule

`default_nettype wire
