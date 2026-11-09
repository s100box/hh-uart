// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
`timescale 1ns / 1ps
//============================================================================
// tb_usb_serial_chan -- self-checking bench for the host<->USB bridge channel.
//
//   Drives the host side of usb_serial_if (the adapter's job) and the stack's
//   register window, and proves the full byte loop:
//     host TX byte -> TX FIFO -> stack reads DATA_TX -> stack echoes to DATA_RX ->
//     RX FIFO -> host reads RX -> compare.
//   Also checks TX backpressure (FIFO full), RX backpressure (the stack
//   overfilling the RX FIFO), the baud and modem-control sidebands, and a
//   reset mid-run.
//
//   Run twice: default = SAME_CLOCK (one clock), -DASYNC_MODE = independent
//   host/stack clocks (exercises the async-FIFO CDC + the baud toggle handshake).
//============================================================================
`default_nettype none

module tb_usb_serial_chan;

`ifdef ASYNC_MODE
  localparam bit SC = 1'b0;
  localparam string ModeName = "ASYNC (SAME_CLOCK=0)";
`else
  localparam bit SC = 1'b1;
  localparam string ModeName = "SYNC (SAME_CLOCK=1)";
`endif
  localparam int DEPTH = 16;

  logic host_clk = 1'b0;
  logic v_clk = 1'b0;
  logic host_nrst = 1'b0;
  logic v_nrst = 1'b0;

  // host_clk ~ 100 MHz; v_clk equal in sync mode, ~62.5 MHz (coprime-ish) async.
  always #5 host_clk = ~host_clk;
  generate
    if (SC) begin : g_oneclk
      always_comb v_clk = host_clk;  // tie the two domains together
    end else begin : g_twoclk
      always #8 v_clk = ~v_clk;  // different rate -> real CDC
    end
  endgenerate

  usb_serial_if us ();

  logic [2:0] v_addr;
  logic v_we, v_re;
  logic [7:0] v_wdata;
  logic [7:0] v_rdata;

  usb_serial_chan #(
      .DEPTH     (DEPTH),
      .SAME_CLOCK(SC)
  ) dut (
      .host_clk_i (host_clk),
      .host_nrst_i(host_nrst),
      .us         (us),
      .v_clk_i    (v_clk),
      .v_nrst_i   (v_nrst),
      .v_addr_i   (v_addr),
      .v_we_i     (v_we),
      .v_re_i     (v_re),
      .v_wdata_i  (v_wdata),
      .v_rdata_o  (v_rdata)
  );

  localparam logic [2:0] RegStatus = 3'd0;
  localparam logic [2:0] RegDataTx = 3'd1;
  localparam logic [2:0] RegDataRx = 3'd2;
  localparam logic [2:0] RegBaud0 = 3'd3;

  integer errors = 0;
  task automatic check(input logic cond, input string what);
    begin
      if (cond !== 1'b1) begin
        $display("FAIL: %s", what);
        errors = errors + 1;
      end
    end
  endtask

  // ---- Host-side driver (host_clk domain) ----------------------------------
  task automatic host_send(input logic [7:0] b);
    begin
      @(negedge host_clk);
      us.tx_data  = b;
      us.tx_valid = 1'b1;
      // Sample ready at the consuming edge, before registered FIFO flags
      // change. A ready rise after that edge did not accept this byte yet.
      do @(posedge host_clk); while (us.tx_ready !== 1'b1);
      @(negedge host_clk);
      us.tx_valid = 1'b0;
    end
  endtask

  // Take the head byte while rx_valid is high, then assert rx_ready for exactly
  // one clock edge so the FIFO pops that byte and no other.
  task automatic host_recv(output logic [7:0] b);
    begin
      @(negedge host_clk);
      while (!us.rx_valid) @(negedge host_clk);
      b           = us.rx_data;
      us.rx_ready = 1'b1;
      @(negedge host_clk);
      us.rx_ready = 1'b0;
    end
  endtask

  // ---- Stack-side register access (v_clk domain) ----------------------------
  task automatic v_read(input logic [2:0] a, output logic [7:0] d);
    begin
      @(negedge v_clk);
      v_addr = a;
      v_re   = 1'b1;
      #1 d = v_rdata;  // combinational read
      @(posedge v_clk);  // pop/ack happens here
      @(negedge v_clk);
      v_re = 1'b0;
    end
  endtask

  task automatic v_write(input logic [2:0] a, input logic [7:0] d);
    begin
      @(negedge v_clk);
      v_addr  = a;
      v_wdata = d;
      v_we    = 1'b1;
      @(posedge v_clk);
      @(negedge v_clk);
      v_we = 1'b0;
    end
  endtask

  // Stack echo pump: whenever a host->USB byte is available, read it and push it
  // straight back as a USB->host byte. Gated by pump_on -- when off it parks
  // WITHOUT touching the v_ bus, so the main stimulus can own it exclusively.
  logic pump_on;
  initial begin
    pump_on = 1'b0;
    v_addr  = 3'd0;
    v_we    = 1'b0;
    v_re    = 1'b0;
    v_wdata = 8'h00;
    forever begin
      logic [7:0] st, by;
      if (!pump_on) begin
        @(posedge v_clk);  // parked: do not drive v_re/v_we
      end else begin
        v_read(RegStatus, st);
        if (st[0]) begin  // TX_AVAIL
          v_read(RegDataTx, by);
          v_write(RegDataRx, by);
        end
      end
    end
  end

  // ---- Stimulus ------------------------------------------------------------
  localparam int NLoop = 64;
  logic [7:0] sent[NLoop];
  logic [7:0] got;
  integer i;

  initial begin
    us.tx_data  = 8'h00;
    us.tx_valid = 1'b0;
    us.rx_ready = 1'b0;
    us.baud_div = 32'h0;
    us.baud_stb = 1'b0;
    us.mctl_out = 2'b00;

    host_nrst   = 1'b0;
    v_nrst      = 1'b0;
    repeat (8) @(posedge host_clk);
    host_nrst = 1'b1;
    v_nrst    = 1'b1;
    repeat (8) @(posedge host_clk);

    $display("--- tb_usb_serial_chan: %s ---", ModeName);

    // Test 1: idle status -- nothing available, RX has space.
    begin
      logic [7:0] st;
      v_read(RegStatus, st);
      check(st[0] === 1'b0, "TX_AVAIL low at idle");
      check(st[1] === 1'b1, "RX_SPACE high at idle");
    end

    // Test 2: full byte loop through the stack echo pump. The first two DEPTH
    // runs write every FIFO cell all-ones then all-zeros; the rest walk a one
    // and a zero through the byte and add random fill.
    pump_on = 1'b1;
    for (i = 0; i < NLoop; i++) begin
      if (i < DEPTH) sent[i] = 8'hFF;
      else if (i < 2 * DEPTH) sent[i] = 8'h00;
      else if (i < 2 * DEPTH + 8) sent[i] = 8'h01 << (i - 2 * DEPTH);
      else if (i < 2 * DEPTH + 16) sent[i] = ~(8'h01 << (i - 2 * DEPTH - 8));
      else sent[i] = 8'($urandom);
    end
    fork
      begin : sender
        for (i = 0; i < NLoop; i++) host_send(sent[i]);
      end
      begin : receiver
        for (int k = 0; k < NLoop; k++) begin
          host_recv(got);
          check(got === sent[k], $sformatf("loop byte %0d (exp %02h got %02h)", k, sent[k], got));
        end
      end
    join

    // Test 3: TX backpressure -- pause the pump, overfill the TX FIFO.
    pump_on = 1'b0;
    repeat (20) @(posedge host_clk);  // let the pump settle/idle
    begin
      integer accepted;
      accepted = 0;
      // Offer DEPTH+4 distinct bytes, each for exactly one clock edge, without
      // draining: exactly DEPTH are taken and ready is low afterwards.
      for (i = 0; i < DEPTH + 4; i++) begin
        @(negedge host_clk);
        us.tx_data  = 8'hA0 + i[7:0];
        us.tx_valid = 1'b1;
        #1;
        if (us.tx_ready) accepted = accepted + 1;  // this edge takes it
        @(negedge host_clk);
        us.tx_valid = 1'b0;
      end
      check(accepted == DEPTH, $sformatf("TX FIFO accepts exactly DEPTH (accepted=%0d)", accepted));
      check(us.tx_ready === 1'b0, "tx_ready low when TX FIFO full");
    end
    // Drain the backpressure bytes via the pump: they land in the RX FIFO, and
    // the host must read back exactly the ones the TX FIFO accepted, in order.
    pump_on = 1'b1;
    repeat (400) @(posedge host_clk);
    pump_on = 1'b0;
    repeat (20) @(posedge v_clk);
    begin
      integer drained;
      drained = 0;
      while (us.rx_valid) begin
        host_recv(got);
        check(got === 8'hA0 + drained[7:0], $sformatf(
              "backpressure byte %0d (exp %02h got %02h)", drained, 8'hA0 + drained[7:0], got));
        drained = drained + 1;
      end
      check(drained == DEPTH, $sformatf("backpressure: %0d bytes drained (exp %0d)", drained, DEPTH
            ));
    end

    // Test 3b: RX backpressure -- the stack overfills the RX FIFO while the host
    // is not reading. RX_SPACE must drop at DEPTH, the extra writes are
    // dropped, and the host then drains exactly DEPTH bytes in order.
    pump_on = 1'b0;
    repeat (20) @(posedge v_clk);
    begin
      logic [7:0] st;
      for (i = 0; i < DEPTH + 2; i++) v_write(RegDataRx, 8'h30 + i[7:0]);
      repeat (8) @(posedge v_clk);
      v_read(RegStatus, st);
      check(st[1] === 1'b0, "RX_SPACE low when the RX FIFO is full");
      for (int k = 0; k < DEPTH; k++) begin
        host_recv(got);
        check(got === 8'h30 + k[7:0], $sformatf(
              "rx fill byte %0d (exp %02h got %02h)", k, 8'h30 + k[7:0], got));
      end
      repeat (20) @(posedge host_clk);
      check(us.rx_valid === 1'b0, "RX FIFO empty after draining DEPTH bytes");
      v_read(RegStatus, st);
      check(st[1] === 1'b1, "RX_SPACE high again after the drain");
    end

    // Test 4: baud sideband crossing. Stop the pump so the main owns the v_ bus.
    // Three divisor words in turn: all-ones, all-zeros (every bit of the word
    // and the toggle both ways), then a real one.
    pump_on = 1'b0;
    repeat (20) @(posedge v_clk);
    begin
      logic [7:0] st, b0, b1, b2, b3;
      logic [31:0] words[3];
      words[0] = 32'hFFFF_FFFF;
      words[1] = 32'h0000_0000;
      words[2] = 32'h0001_C200;  // 115200-ish divisor word
      for (int w = 0; w < 3; w++) begin
        @(negedge host_clk);
        us.baud_div = words[w];
        us.baud_stb = 1'b1;
        @(negedge host_clk);
        us.baud_stb = 1'b0;
        // wait for the stack side to flag it
        repeat (30) @(posedge v_clk);
        v_read(RegStatus, st);
        check(st[2] === 1'b1, $sformatf("BAUD_PENDING set after host baud_stb (word %0d)", w));
        v_read(RegBaud0, b0);
        v_read(3'd4, b1);
        v_read(3'd5, b2);
        v_read(3'd6, b3);  // BAUD3 read clears pending
        check({b3, b2, b1, b0} === words[w], $sformatf(
              "baud divisor word %0d crossed intact (%08h)", w, {b3, b2, b1, b0}));
        repeat (4) @(posedge v_clk);
        v_read(RegStatus, st);
        check(st[2] === 1'b0, $sformatf("BAUD_PENDING cleared after BAUD3 read (word %0d)", w));
      end
    end

    // Test 5: modem-control sideband. Outbound: host RTS/DTR change sets
    // MCTL_PENDING; the MCTL read returns the levels and clears the flag.
    // Inbound: the MCTL write's status nibble crosses to us.mctl_in (which
    // resets ASSERTED, 4'b1111).
    begin
      logic [7:0] m;
      logic [1:0] seq[4];
      logic [7:0] wr [3];
      check(us.mctl_in === 4'b1111, "mctl_in resets asserted");
      // Walk RTS/DTR through every transition: 00 -> 01 -> 11 -> 10 -> 00.
      seq[0] = 2'b01;
      seq[1] = 2'b11;
      seq[2] = 2'b10;
      seq[3] = 2'b00;
      for (int k = 0; k < 4; k++) begin
        @(negedge host_clk);
        us.mctl_out = seq[k];
        repeat (30) @(posedge v_clk);
        v_read(3'd7, m);
        check(m[7] === 1'b1, $sformatf("MCTL_PENDING set after mctl_out -> %b", seq[k]));
        check(m[1:0] === seq[k], $sformatf("MCTL read returns the host RTS/DTR levels %b", seq[k]));
        check(m[6:2] === 5'd0, "MCTL read: bits 6:2 zero");
        repeat (4) @(posedge v_clk);
        v_read(3'd7, m);
        check(m[7] === 1'b0, "MCTL_PENDING cleared by the read");
      end
      // Adapter status nibble: 0101, then 1010 with the ignored upper bits set,
      // then all clear; each must cross to mctl_in.
      wr[0] = 8'h05;  // DCD=0, RI=1, DSR=0, CTS=1
      wr[1] = 8'h4A;  // DCD=1, RI=0, DSR=1, CTS=0 (bit 6 set: ignored)
      wr[2] = 8'h00;
      for (int k = 0; k < 3; k++) begin
        v_write(3'd7, wr[k]);
        repeat (30) @(posedge host_clk);
        check(us.mctl_in === wr[k][3:0], $sformatf(
              "adapter status nibble %h crossed to mctl_in (%b)", wr[k], us.mctl_in));
      end
    end

    // Test 6: reset mid-run. Leave a byte in each FIFO and the adapter status
    // cleared, pull both resets, and check the channel comes back empty with
    // mctl_in asserted again.
    begin
      logic [7:0] st;
      host_send(8'h5A);
      v_write(RegDataRx, 8'hA5);
      repeat (20) @(posedge v_clk);
      v_read(RegStatus, st);
      check(st[0] === 1'b1, "pre-reset: TX_AVAIL set");
      check(us.rx_valid === 1'b1, "pre-reset: rx_valid set");
      @(negedge host_clk);
      host_nrst = 1'b0;
      v_nrst    = 1'b0;
      repeat (8) @(posedge host_clk);
      repeat (8) @(posedge v_clk);
      @(negedge host_clk);
      host_nrst = 1'b1;
      v_nrst    = 1'b1;
      repeat (20) @(posedge v_clk);
      v_read(RegStatus, st);
      check(st === 8'h02, $sformatf("post-reset STATUS idle (got %02h)", st));
      check(us.rx_valid === 1'b0, "post-reset: rx_valid clear");
      check(us.tx_ready === 1'b1, "post-reset: tx_ready set");
      check(us.mctl_in === 4'b1111, "post-reset: mctl_in asserted");
      v_read(3'd7, st);
      check(st === 8'h00, "post-reset: MCTL clear");
      // The channel still works: one byte round trip through the pump.
      pump_on = 1'b1;
      host_send(8'h96);
      host_recv(got);
      check(got === 8'h96, "post-reset: byte round trip");
      pump_on = 1'b0;
    end

    if (errors == 0) $display("=== tb_usb_serial_chan %s: ALL CHECKS PASS ===", ModeName);
    else $display("=== tb_usb_serial_chan %s: %0d FAILURE(S) ===", ModeName, errors);
    if (errors != 0) $fatal(1, "tb_usb_serial_chan FAILED");
    $finish;
  end

  initial begin
    #2_000_000;
    $display("WATCHDOG timeout");
    $fatal(1, "tb_usb_serial_chan watchdog");
  end

endmodule

`default_nettype wire
