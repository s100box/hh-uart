// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Transaction scoreboards attach to the external producers/consumers of the
// real channel, optionally through the real ACIA adapter. Digital clocks may
// pause independently after both domains have observed startup reset.
module channel_queue_props #(
    parameter bit SAME_CLOCK = 0
) (
    input wire logic        formal_clk,
    wr_clk,
    rd_clk,
    nrst,
    input wire logic        write_request,
    read_request,
    full,
    empty,
    input wire logic [ 7:0] data_in,
    data_out,
    input wire logic [ 2:0] actual_written,
    actual_read,
    input wire logic [ 2:0] wr_meta,
    wr_seen,
    rd_meta,
    rd_seen,
    input wire logic [31:0] memory
);
  logic [2:0] written, consumed;
  logic [7:0] reference[4];
  logic [3:0] initialized;
  always @(posedge wr_clk) begin
    if (!nrst) begin
      written     <= 0;
      initialized <= 0;
    end else if (write_request && !full) begin
      reference[written[1:0]]   <= data_in;
      initialized[written[1:0]] <= 1;
      written                   <= written + 1'b1;
    end
  end
  always @(posedge rd_clk) begin
    if (!nrst) consumed <= 0;
    else if (read_request && !empty) begin
      assert (initialized[consumed[1:0]]);
      assert (data_out == reference[consumed[1:0]]);
      consumed <= consumed + 1'b1;
    end
  end
  wire [2:0] used = written - consumed;
  logic [2:0] read_for_full, write_for_empty;
  always @(posedge wr_clk)
    if (!nrst) read_for_full <= 0;
    else read_for_full <= rd_seen;
  always @(posedge rd_clk)
    if (!nrst) write_for_empty <= 0;
    else write_for_empty <= wr_seen;
  always @(posedge formal_clk)
    if (nrst) begin
      assert (written == actual_written);
      assert (consumed == actual_read);
      assert (used <= 4);
      if (!full) assert (used < 4);
      if (!empty) assert (used != 0);
      assert (3'(written - wr_meta) <= 4);
      assert (3'(written - wr_seen) <= 4);
      assert (3'(wr_meta - wr_seen) <= 4);
      assert (3'(wr_seen - consumed) <= 4);
      assert (3'(consumed - rd_meta) <= 4);
      assert (3'(consumed - rd_seen) <= 4);
      assert (3'(rd_meta - rd_seen) <= 4);
      assert (3'(written - rd_seen) <= 4);
      if (!SAME_CLOCK) begin
        assert (3'(rd_seen - read_for_full) <= 4);
        assert (3'(consumed - read_for_full) <= 4);
        assert (3'(written - read_for_full) <= 4);
        assert (full == (3'(written - read_for_full) == 4));
        assert (3'(wr_seen - write_for_empty) <= 4);
        assert (3'(written - write_for_empty) <= 4);
        assert (3'(write_for_empty - consumed) <= 4);
        assert (empty == (write_for_empty == consumed));
        assert ({2'b0, used} + {2'b0, 3'(consumed-rd_meta)} +
              {2'b0, 3'(rd_meta-rd_seen)} + {2'b0, 3'(rd_seen-read_for_full)} <= 4);
        assert ({2'b0, 3'(written-wr_meta)} + {2'b0, 3'(wr_meta-wr_seen)} +
              {2'b0, 3'(wr_seen-write_for_empty)} + {2'b0, 3'(write_for_empty-consumed)} ==
              {2'b0, used});
      end
      cover (full && written != 0);
      cover (written[2] && consumed[2]);
    end
  for (genvar i = 0; i < 4; i++) begin : g_memory
    wire [1:0] distance = 2'(i) - consumed[1:0];
    always @(posedge formal_clk)
      if (nrst) begin
        if (initialized[i]) assert (memory[8*i+:8] == reference[i]);
        if (3'(distance) < used) assert (initialized[i]);
      end
  end
endmodule

module channel_formal #(
    parameter bit SAME_CLOCK = 0,
    parameter bit ADAPTER = 0
) (
    (* gclk *) input wire logic        formal_clk,
               input wire logic        advance_host,
    advance_stack,
               input wire logic [ 7:0] tx_byte,
    stack_write_data,
               input wire logic        tx_valid,
    rx_full,
               input wire logic [ 2:0] address,
               input wire logic        stack_write,
    stack_read,
               input wire logic [31:0] baud_divisor,
               input wire logic        baud_strobe,
               input wire logic [ 1:0] modem_out
);
  logic host_clk = 0, stack_free = 0;
  logic [2:0] startup = 0;
  wire nrst = &startup;
  always @(posedge formal_clk) begin
    if (!nrst) startup <= startup + 1'b1;
    if (!nrst || advance_host) host_clk <= !host_clk;
    if (!nrst || advance_stack) stack_free <= !stack_free;
  end
  wire stack_clk = SAME_CLOCK ? host_clk : stack_free;
  usb_serial_if us ();
  wire [7:0] stack_data, rx_byte;
  wire tx_consumed, rx_strobe;
  if (ADAPTER) begin : g_adapter
    acia6551_to_usbserial adapter (
        .tx_byte_i    (tx_byte),
        .tx_valid_i   (tx_valid),
        .tx_consumed_o(tx_consumed),
        .rx_byte_o    (rx_byte),
        .rx_strobe_o  (rx_strobe),
        .rx_full_i    (rx_full),
        .us           (us)
    );
  end else begin : g_raw
    assign us.tx_data  = tx_byte;
    assign us.tx_valid = tx_valid;
    assign tx_consumed = tx_valid && us.tx_ready;
    assign us.rx_ready = !rx_full;
    assign rx_strobe   = us.rx_valid && !rx_full;
    assign rx_byte     = us.rx_data;
    assign us.baud_div = baud_divisor;
    assign us.baud_stb = baud_strobe;
    assign us.mctl_out = modem_out;
  end
  usb_serial_chan #(
      .DEPTH     (4),
      .SAME_CLOCK(SAME_CLOCK)
  ) dut (
      .host_clk_i (host_clk),
      .host_nrst_i(nrst),
      .us         (us),
      .v_clk_i    (stack_clk),
      .v_nrst_i   (nrst),
      .v_addr_i   (address),
      .v_we_i     (stack_write),
      .v_re_i     (stack_read),
      .v_wdata_i  (stack_write_data),
      .v_rdata_o  (stack_data)
  );
  wire [2:0] tx_written, tx_read, rx_written, rx_read;
  wire [2:0] tx_wm, tx_ws, tx_rm, tx_rs, rx_wm, rx_ws, rx_rm, rx_rs;
  function automatic logic [2:0] ungray3(input logic [2:0] gray);
    ungray3 = {gray[2], ^gray[2:1], ^gray};
  endfunction
  if (SAME_CLOCK) begin : g_sync
    assign tx_written = dut.u_tx_fifo.g_sync.wr_bin_q;
    assign tx_read    = dut.u_tx_fifo.g_sync.rd_bin_q;
    assign rx_written = dut.u_rx_fifo.g_sync.wr_bin_q;
    assign rx_read    = dut.u_rx_fifo.g_sync.rd_bin_q;
    assign tx_wm      = tx_written;
    assign tx_ws      = tx_written;
    assign tx_rm      = tx_read;
    assign tx_rs      = tx_read;
    assign rx_wm      = rx_written;
    assign rx_ws      = rx_written;
    assign rx_rm      = rx_read;
    assign rx_rs      = rx_read;
  end else begin : g_async
    assign tx_written = dut.u_tx_fifo.g_async.wr_bin_q;
    assign tx_read    = dut.u_tx_fifo.g_async.rd_bin_q;
    assign rx_written = dut.u_rx_fifo.g_async.wr_bin_q;
    assign rx_read    = dut.u_rx_fifo.g_async.rd_bin_q;
    assign tx_wm      = ungray3(dut.u_tx_fifo.g_async.wr_gray_m);
    assign tx_ws      = ungray3(dut.u_tx_fifo.g_async.wr_gray_s);
    assign tx_rm      = ungray3(dut.u_tx_fifo.g_async.rd_gray_m);
    assign tx_rs      = ungray3(dut.u_tx_fifo.g_async.rd_gray_s);
    assign rx_wm      = ungray3(dut.u_rx_fifo.g_async.wr_gray_m);
    assign rx_ws      = ungray3(dut.u_rx_fifo.g_async.wr_gray_s);
    assign rx_rm      = ungray3(dut.u_rx_fifo.g_async.rd_gray_m);
    assign rx_rs      = ungray3(dut.u_rx_fifo.g_async.rd_gray_s);
    always @(posedge formal_clk)
      if (nrst) begin
        assert (dut.u_tx_fifo.g_async.wr_gray_q == (tx_written ^ (tx_written >> 1)));
        assert (dut.u_tx_fifo.g_async.rd_gray_q == (tx_read ^ (tx_read >> 1)));
        assert (dut.u_rx_fifo.g_async.wr_gray_q == (rx_written ^ (rx_written >> 1)));
        assert (dut.u_rx_fifo.g_async.rd_gray_q == (rx_read ^ (rx_read >> 1)));
      end
  end
  channel_queue_props #(
      .SAME_CLOCK(SAME_CLOCK)
  ) tx_scoreboard (
      .formal_clk(formal_clk),
      .wr_clk(host_clk),
      .rd_clk(stack_clk),
      .nrst(nrst),
      .write_request(tx_valid),
      .read_request(stack_read && address == 1),
      .full(!us.tx_ready),
      .empty(dut.tx_empty),
      .data_in(tx_byte),
      .data_out(stack_data),
      .actual_written(tx_written),
      .actual_read(tx_read),
      .wr_meta(tx_wm),
      .wr_seen(tx_ws),
      .rd_meta(tx_rm),
      .rd_seen(tx_rs),
      .memory({
        dut.u_tx_fifo.mem[3], dut.u_tx_fifo.mem[2], dut.u_tx_fifo.mem[1], dut.u_tx_fifo.mem[0]
      })
  );
  channel_queue_props #(
      .SAME_CLOCK(SAME_CLOCK)
  ) rx_scoreboard (
      .formal_clk(formal_clk),
      .wr_clk(stack_clk),
      .rd_clk(host_clk),
      .nrst(nrst),
      .write_request(stack_write && address == 2),
      .read_request(!rx_full),
      .full(dut.rx_full),
      .empty(!us.rx_valid),
      .data_in(stack_write_data),
      .data_out(rx_byte),
      .actual_written(rx_written),
      .actual_read(rx_read),
      .wr_meta(rx_wm),
      .wr_seen(rx_ws),
      .rd_meta(rx_rm),
      .rd_seen(rx_rs),
      .memory({
        dut.u_rx_fifo.mem[3], dut.u_rx_fifo.mem[2], dut.u_rx_fifo.mem[1], dut.u_rx_fifo.mem[0]
      })
  );
  // Sidebands are unbuffered digital crossings. This checks their exact
  // sampling/acknowledgement behavior, not delivery of arbitrarily fast updates.
  logic [31:0] sent_baud, received_baud;
  logic baud_toggle, baud_pending, modem_pending;
  logic [2:0] request_pipe;
  logic [1:0] out_meta, out_sync, out_previous;
  logic [3:0] in_latch, in_meta, in_sync;
  always @(posedge host_clk) begin
    if (!nrst) begin
      sent_baud   <= 0;
      baud_toggle <= 0;
      in_meta     <= 15;
      in_sync     <= 15;
    end else begin
      if (us.baud_stb) begin
        sent_baud   <= us.baud_div;
        baud_toggle <= !baud_toggle;
      end
      in_meta <= in_latch;
      in_sync <= in_meta;
    end
  end
  always @(posedge stack_clk) begin
    if (!nrst) begin
      request_pipe  <= 0;
      received_baud <= 0;
      baud_pending  <= 0;
      out_meta      <= 0;
      out_sync      <= 0;
      out_previous  <= 0;
      modem_pending <= 0;
      in_latch      <= 15;
    end else begin
      request_pipe <= {request_pipe[1:0], baud_toggle};
      if (request_pipe[1] != request_pipe[2]) begin
        received_baud <= sent_baud;
        baud_pending  <= 1;
      end else if (stack_read && address == 6) baud_pending <= 0;
      out_meta     <= us.mctl_out;
      out_sync     <= out_meta;
      out_previous <= out_sync;
      if (out_sync != out_previous) modem_pending <= 1;
      else if (stack_read && address == 7) modem_pending <= 0;
      if (stack_write && address == 7) in_latch <= stack_write_data[3:0];
    end
  end
  always @(posedge formal_clk)
    if (nrst) begin
      assert (tx_consumed == (tx_valid && us.tx_ready));
      assert (rx_strobe == (us.rx_valid && !rx_full));
      assert (us.rx_err == 0);
      assert (us.mctl_in == in_sync);
      assert (dut.baud_hold_q == sent_baud);
      assert (dut.baud_tog_q == baud_toggle);
      assert ({dut.baud_tog_s2, dut.baud_tog_s, dut.baud_tog_m} == request_pipe);
      assert (dut.baud_word_q == received_baud);
      assert (dut.baud_pending_q == baud_pending);
      assert ({dut.mo_prev, dut.mo_s, dut.mo_m} == {out_previous, out_sync, out_meta});
      assert (dut.mctl_pending_q == modem_pending);
      assert ({dut.mi_s, dut.mi_m, dut.mi_q} == {in_sync, in_meta, in_latch});
      if (address == 0) assert (stack_data == {5'd0, baud_pending, !dut.rx_full, !dut.tx_empty});
      if (address >= 3 && address <= 6) assert (stack_data == received_baud[8*(address-3)+:8]);
      if (address == 7) assert (stack_data == {modem_pending, 5'd0, out_sync});
      if (ADAPTER) begin
        assert (us.baud_div == 0 && !us.baud_stb && us.mctl_out == 3);
      end else cover (baud_pending && stack_read && address == 6 && received_baud == 32'h12345678);
      cover (modem_pending && stack_read && address == 7);
`ifdef FORMAL_NEGATIVE
      assert (!tx_consumed);
`endif
    end
endmodule
