// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Arbitrarily paused digital clocks. A reset request holds both domains in
// reset while forced clocks let each observe the flush. Unilateral reset and
// analog metastability are outside the FIFO's supported contract.
module usb_fifo_formal #(
    parameter bit SAME_CLOCK = 1'b0,
    parameter int unsigned DEPTH = 4
) (
    (* gclk *) input wire logic       formal_clk,
               input wire logic       advance_wr,
               input wire logic       advance_rd,
               input wire logic       restart,
               input wire logic       we,
               input wire logic       re,
               input wire logic [7:0] data
);
  localparam int unsigned Aw = $clog2(DEPTH);
  localparam int unsigned Pw = Aw + 1;
  logic wr_clk = 0, rd_free = 0;
  logic [2:0] startup = 0;
  wire nrst = &startup;
  always @(posedge formal_clk) begin
    if (restart) startup <= 0;
    else if (!nrst) startup <= startup + 1'b1;
    if (!nrst || advance_wr) wr_clk <= !wr_clk;
    if (!nrst || advance_rd) rd_free <= !rd_free;
  end
  wire rd_clk = SAME_CLOCK ? wr_clk : rd_free;
  wire full, empty;
  wire [7:0] head;
  usb_serial_fifo #(
      .DEPTH     (DEPTH),
      .SAME_CLOCK(SAME_CLOCK)
  ) dut (
      .wr_clk_i (wr_clk),
      .wr_nrst_i(nrst),
      .we_i     (we),
      .wdata_i  (data),
      .full_o   (full),
      .rd_clk_i (rd_clk),
      .rd_nrst_i(nrst),
      .re_i     (re),
      .rdata_o  (head),
      .empty_o  (empty)
  );

  // Independent transaction counters define accepted ordering, not flag latency.
  logic [Pw-1:0] written, read;
  logic [7:0] reference[DEPTH];
  logic [DEPTH-1:0] initialized;
  always @(posedge wr_clk) begin
    if (!nrst) begin
      written     <= 0;
      initialized <= 0;
    end else if (we && !full) begin
      written                      <= written + 1'b1;
      reference[written[Aw-1:0]]   <= data;
      initialized[written[Aw-1:0]] <= 1;
    end
  end
  always @(posedge rd_clk) begin
    if (!nrst) read <= 0;
    else if (re && !empty) begin
      read <= read + 1'b1;
      assert (initialized[read[Aw-1:0]]);
      assert (head == reference[read[Aw-1:0]]);
    end
  end
  wire [Pw-1:0] used = written - read;
  if (SAME_CLOCK) begin : g_sync_check
    always @(posedge formal_clk)
      if (nrst) begin
        assert (dut.g_sync.wr_bin_q == written);
        assert (dut.g_sync.rd_bin_q == read);
      end
  end else begin : g_async_check
    function automatic logic [Pw-1:0] ungray(input logic [Pw-1:0] gray);
      ungray = gray;
      for (int bitno = int'(Pw) - 2; bitno >= 0; bitno--)
      ungray[bitno] = ungray[bitno+1] ^ gray[bitno];
    endfunction
    wire [Pw-1:0] wm = ungray(dut.g_async.wr_gray_m);
    wire [Pw-1:0] ws = ungray(dut.g_async.wr_gray_s);
    wire [Pw-1:0] rm = ungray(dut.g_async.rd_gray_m);
    wire [Pw-1:0] rs = ungray(dut.g_async.rd_gray_s);
    logic [Pw-1:0] read_for_full, write_for_empty;
    always @(posedge wr_clk)
      if (!nrst) read_for_full <= 0;
      else read_for_full <= rs;
    always @(posedge rd_clk)
      if (!nrst) write_for_empty <= 0;
      else write_for_empty <= ws;
    always @(posedge formal_clk)
      if (nrst) begin
        assert (dut.g_async.wr_bin_q == written);
        assert (dut.g_async.rd_bin_q == read);
        assert (dut.g_async.wr_gray_q == (written ^ (written >> 1)));
        assert (dut.g_async.rd_gray_q == (read ^ (read >> 1)));
        // Synchronized pointers are ordered historical observations. Each
        // distance uses the extended pointer width, including wrap parity.
        assert (Pw'(written - wm) <= DEPTH);
        assert (Pw'(written - ws) <= DEPTH);
        assert (Pw'(wm - ws) <= DEPTH);
        assert (Pw'(ws - read) <= DEPTH);
        assert (Pw'(read - rm) <= DEPTH);
        assert (Pw'(read - rs) <= DEPTH);
        assert (Pw'(rm - rs) <= DEPTH);
        assert (Pw'(written - rs) <= DEPTH);
        assert (Pw'(rs - read_for_full) <= DEPTH);
        assert (Pw'(read - read_for_full) <= DEPTH);
        assert (Pw'(written - read_for_full) <= DEPTH);
        assert (full == (Pw'(written - read_for_full) == DEPTH));
        assert (Pw'(ws - write_for_empty) <= DEPTH);
        assert (Pw'(written - write_for_empty) <= DEPTH);
        assert (Pw'(write_for_empty - read) <= DEPTH);
        assert (empty == (write_for_empty == read));
        // Widen before adding: pairwise modular bounds alone admit a false
        // extra lap at the full/empty wrap boundary in an induction state.
        assert ({2'b0, used} + {2'b0, Pw'(read-rm)} + {2'b0, Pw'(rm-rs)} +
                {2'b0, Pw'(rs-read_for_full)} <= DEPTH);
        assert ({2'b0, Pw'(written-wm)} + {2'b0, Pw'(wm-ws)} +
                {2'b0, Pw'(ws-write_for_empty)} + {2'b0, Pw'(write_for_empty-read)} ==
                {2'b0, used});
      end
  end
  for (genvar i = 0; i < DEPTH; i++) begin : g_memory_check
    wire [Aw-1:0] distance = Aw'(i) - read[Aw-1:0];
    always @(posedge formal_clk)
      if (nrst && initialized[i]) begin
        assert (dut.mem[i] == reference[i]);
      end
    always @(posedge formal_clk)
      if (nrst && Pw'(distance) < used) begin
        assert (initialized[i]);
      end
  end
  logic saw_full = 0, saw_read = 0, flushed_full = 0;
  always @(posedge formal_clk) if (nrst && restart && used != 0) flushed_full <= 1;
  always @(posedge formal_clk)
    if (nrst) begin
      assert (used <= DEPTH);
      if (!full) assert (used < DEPTH);
      if (!empty) assert (used != 0);
      if (full) saw_full <= 1;
      if (read != 0) saw_read <= 1;
      cover (full);
      cover (saw_full && empty && saw_read);
      cover (written[Pw-1] && read[Pw-1]);
      cover (flushed_full && empty && used == 0);
`ifdef FORMAL_NEGATIVE
      assert (used == 0);  // false: a producer can enqueue a byte
`endif
    end
endmodule
