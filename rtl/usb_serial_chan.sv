// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// usb_serial_chan -- the host-UART <-> USB bridge backend.
//
//   A TX/RX byte-FIFO pair: it presents the usb_serial_if `chan` modport to a
//   host UART core (through that core's small adapter) and an eight-register
//   byte window (the v_* ports) to the processor running the USB host stack.
//   The FIFOs give the host the flow control its UART may lack and absorb USB
//   bursts; the optional baud sideband carries the host's divisor word out to
//   the stack so it can match the USB adapter's line rate.
//
//   TX FIFO: host writes (usb_serial_if.tx), the stack drains (read DATA_TX).
//   RX FIFO: the stack fills (write DATA_RX), host reads (usb_serial_if.rx).
//   SAME_CLOCK=1 selects single-clock FIFOs (host_clk_i == v_clk_i); sideband
//   synchronizer pipelines retain their latency in either configuration.
//   See usb_serial_fifo.sv for the reset and physical CDC timing contract.
//
//   Stack register window (v_addr_i[2:0], byte-wide, combinational read):
//     0 STATUS (R): bit0 TX_AVAIL (host->USB FIFO has a byte),
//                   bit1 RX_SPACE (USB->host FIFO not full),
//                   bit2 BAUD_PENDING (host changed the divisor)
//     1 DATA_TX (R): pop the oldest host->USB byte (read advances the FIFO)
//     2 DATA_RX (W): push a USB->host byte
//     3..6 BAUD0..3 (R): the latched 32-bit divisor, LSB first;
//                   reading BAUD3 (addr 6) acknowledges/clears BAUD_PENDING
//     7 MCTL (R): {MCTL_PENDING[7], 5'b0, RTS[1], DTR[0]} -- the host's
//                   outbound modem lines; reading acknowledges/clears the
//                   pending flag (set on any RTS/DTR change).
//       MCTL (W): {4'bx, DCD[3], RI[2], DSR[1], CTS[0]} -- the USB adapter's
//                   modem status toward the host (resets ASSERTED, 4'b1111,
//                   so hosts see ready lines until the stack knows better).
//============================================================================
`default_nettype none

module usb_serial_chan #(
    parameter int unsigned DEPTH = 16,
    parameter bit SAME_CLOCK = 1'b0
) (
    // Host UART core domain.
    input wire logic         host_clk_i,
    input wire logic         host_nrst_i,
          usb_serial_if.chan us,

    // USB host stack domain + register window.
    input  wire logic       v_clk_i,
    input  wire logic       v_nrst_i,
    input  wire logic [2:0] v_addr_i,
    input  wire logic       v_we_i,     // one-cycle write strobe
    input  wire logic       v_re_i,     // one-cycle read strobe
    input  wire logic [7:0] v_wdata_i,
    output logic      [7:0] v_rdata_o
);

  localparam logic [2:0] RegStatus = 3'd0;
  localparam logic [2:0] RegDataTx = 3'd1;
  localparam logic [2:0] RegDataRx = 3'd2;
  localparam logic [2:0] RegBaud0 = 3'd3;
  localparam logic [2:0] RegBaud3 = 3'd6;
  localparam logic [2:0] RegMctl = 3'd7;

  // ---- TX FIFO: host -> stack --------------------------------------------------
  logic tx_full, tx_empty;
  wire        tx_we = us.tx_valid & ~tx_full;  // accept on valid && ready
  wire        tx_re = v_re_i & (v_addr_i == RegDataTx);
  logic [7:0] tx_rdata;

  usb_serial_fifo #(
      .DEPTH     (DEPTH),
      .SAME_CLOCK(SAME_CLOCK)
  ) u_tx_fifo (
      .wr_clk_i (host_clk_i),
      .wr_nrst_i(host_nrst_i),
      .we_i     (tx_we),
      .wdata_i  (us.tx_data),
      .full_o   (tx_full),
      .rd_clk_i (v_clk_i),
      .rd_nrst_i(v_nrst_i),
      .re_i     (tx_re),
      .rdata_o  (tx_rdata),
      .empty_o  (tx_empty)
  );

  assign us.tx_ready = ~tx_full;

  // ---- RX FIFO: stack -> host --------------------------------------------------
  logic rx_full, rx_empty;
  wire        rx_we = v_we_i & (v_addr_i == RegDataRx);
  wire        rx_re = us.rx_valid & us.rx_ready;
  logic [7:0] rx_rdata;

  usb_serial_fifo #(
      .DEPTH     (DEPTH),
      .SAME_CLOCK(SAME_CLOCK)
  ) u_rx_fifo (
      .wr_clk_i (v_clk_i),
      .wr_nrst_i(v_nrst_i),
      .we_i     (rx_we),
      .wdata_i  (v_wdata_i),
      .full_o   (rx_full),
      .rd_clk_i (host_clk_i),
      .rd_nrst_i(host_nrst_i),
      .re_i     (rx_re),
      .rdata_o  (rx_rdata),
      .empty_o  (rx_empty)
  );

  assign us.rx_data  = rx_rdata;
  assign us.rx_valid = ~rx_empty;
  assign us.rx_err   = 2'b00;  // USB-sourced RX has no wire framing errors

  // ---- Baud sideband: host -> stack (held data + event toggle) -----------------
  // There is no return acknowledgment or event queue. The producer must hold
  // baud_hold_q stable until the running stack clock captures it. Rapid strobes
  // can replace the word or cancel two toggle transitions before observation.
  logic [31:0] baud_hold_q;
  logic        baud_tog_q;
  always_ff @(posedge host_clk_i) begin
    if (!host_nrst_i) begin
      baud_hold_q <= 32'd0;
      baud_tog_q  <= 1'b0;
    end else if (us.baud_stb) begin
      baud_hold_q <= us.baud_div;
      baud_tog_q  <= ~baud_tog_q;
    end
  end

  logic baud_tog_m, baud_tog_s, baud_tog_s2;
  logic        baud_pending_q;
  logic [31:0] baud_word_q;
  always_ff @(posedge v_clk_i) begin
    if (!v_nrst_i) begin
      baud_tog_m     <= 1'b0;
      baud_tog_s     <= 1'b0;
      baud_tog_s2    <= 1'b0;
      baud_pending_q <= 1'b0;
      baud_word_q    <= 32'd0;
    end else begin
      baud_tog_m  <= baud_tog_q;
      baud_tog_s  <= baud_tog_m;
      baud_tog_s2 <= baud_tog_s;
      if (baud_tog_s != baud_tog_s2) begin
        baud_word_q    <= baud_hold_q;  // producer-held data; see crossing contract above
        baud_pending_q <= 1'b1;
      end else if (v_re_i && (v_addr_i == RegBaud3)) begin
        baud_pending_q <= 1'b0;  // BAUD3 read acknowledges
      end
    end
  end

  // ---- Modem-control sideband ---------------------------------------------
  // Outbound (host -> stack): 2-FF sync the RTS/DTR levels into the v domain;
  // a change sets MCTL_PENDING, reading MCTL clears it (same ack style as the
  // baud word). Inbound (stack -> host): the MCTL write latches the adapter's
  // status in the v domain; 2-FF sync the levels back to the host. Levels
  // are independent modem indications, not an atomic multi-bit word. A source
  // update can become visible over more than one destination clock.
  logic [1:0] mo_m, mo_s, mo_prev;
  logic mctl_pending_q;
  always_ff @(posedge v_clk_i) begin
    if (!v_nrst_i) begin
      mo_m           <= 2'b00;
      mo_s           <= 2'b00;
      mo_prev        <= 2'b00;
      mctl_pending_q <= 1'b0;
    end else begin
      mo_m    <= us.mctl_out;
      mo_s    <= mo_m;
      mo_prev <= mo_s;
      if (mo_s != mo_prev) mctl_pending_q <= 1'b1;
      else if (v_re_i && (v_addr_i == RegMctl)) mctl_pending_q <= 1'b0;
    end
  end

  logic [3:0] mi_q;  // v-domain latch of the adapter status (resets asserted)
  always_ff @(posedge v_clk_i) begin
    if (!v_nrst_i) mi_q <= 4'b1111;
    else if (v_we_i && (v_addr_i == RegMctl)) mi_q <= v_wdata_i[3:0];
  end
  logic [3:0] mi_m, mi_s;
  always_ff @(posedge host_clk_i) begin
    if (!host_nrst_i) begin
      mi_m <= 4'b1111;
      mi_s <= 4'b1111;
    end else begin
      mi_m <= mi_q;
      mi_s <= mi_m;
    end
  end
  assign us.mctl_in = mi_s;

  // ---- Stack register read mux (combinational) ---------------------------------
  always_comb begin
    unique case (v_addr_i)
      RegStatus: v_rdata_o = {5'd0, baud_pending_q, ~rx_full, ~tx_empty};
      RegDataTx: v_rdata_o = tx_rdata;
      RegBaud0:  v_rdata_o = baud_word_q[7:0];
      3'd4:      v_rdata_o = baud_word_q[15:8];
      3'd5:      v_rdata_o = baud_word_q[23:16];
      RegBaud3:  v_rdata_o = baud_word_q[31:24];
      RegMctl:   v_rdata_o = {mctl_pending_q, 5'd0, mo_s};
      default:   v_rdata_o = 8'h00;
    endcase
  end

endmodule

`default_nettype wire
