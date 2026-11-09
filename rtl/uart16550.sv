// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// uart16550 -- synthesizable 8250/16450/16550/16550A UART.
//
//   The National/TI 8250-family register set: register model, IIR priority,
//   FIFOs, THRE re-arm, character timeout, loopback, modem deltas, variant
//   gating. The serial wire is driven by uart_tx_core / uart_rx_core (5-8
//   data bits, none/even/odd/mark/space parity, 1/2 stop, break) and the baud
//   by uart_baud_dds.
//
//   CPU-agnostic: the SoC does the port decode and presents an access as
//   {cs_i, addr_i[2:0], rd_i|wr_i, din_i} -> dout_o. The level int_o is
//   OUT2-gated (open in loopback, so a loopback self-test sees its
//   interrupts); int_vec_o carries IRQ_VECTOR for a Z80 / Z180 IM2 acknowledge
//   (ignored under IM1, where the bare level line suffices).
//
//   VARIANT selects the family member: 0=8250 (no FIFO, no scratch), 1=16450
//   (scratch, no FIFO), 2=16550 (reports a FIFO but does not buffer -- the
//   silicon bug), 3=16550A (working 16-byte FIFOs -- default). On 8250/16450
//   fifo_en can never assert, so the FIFO arrays constant-propagate away.
//
//   Line status: OE/PE/FE/BI are sticky until an LSR read (a flag raised in
//   the clock of the read survives it). In FIFO mode a character's PE/FE/BI
//   reach the LSR when it reaches the top of the RX FIFO, so a driver reads
//   LSR then RBR and attributes the flags to that byte; FIFOE is set while any
//   errored character is in the RX FIFO and clears on an LSR read once none
//   is. FCR1/FCR2, and any change of FCR0, clear the FIFOs.
//
//   Transmit: uart_tx_core runs without its holding buffer (HOLD = 0), so a
//   waiting byte stays in the THR or the 16-byte TX FIFO until the shifter
//   takes it at its start bit: THR (or FIFO) + TSR, as on the parts. THRE
//   rises as the last byte enters the shifter, a THR write while THRE is low
//   replaces the waiting byte, and a TX FIFO clear discards every waiting
//   byte. In FIFO mode the THRE interrupt is held back by a character time
//   less the stop bit unless two bytes have been queued at once, or FCR0
//   changed, since THRE last rose.
//   Loopback forces RTS#/DTR# inactive and holds TXD marking.
//
//   Baud is REF_HZ-relative with CLK_HZ matching the input clock. A DDS accumulator
//   synthesizes the exact REF_HZ crystal rate from clk_i (average rate exact,
//   <= one clk_i of jitter), then divide-by-(16*divisor) gives one tick per
//   serial bit. A design can use another core clock by changing CLK_HZ while
//   retaining the firmware divisor mapping (baud = REF_HZ / (16*divisor)).
//
//   Self-checking bench: sim/tb_uart16550.sv (in `ninja bench`).
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart16550 #(
    // Family member: 0 = 8250, 1 = 16450, 2 = 16550 (buggy FIFO), 3 = 16550A.
    parameter int unsigned VARIANT = 3,
    // Core (clk_i) frequency and the UART reference crystal. The baud rate is
    // CLK_HZ must match the actual input clock; the firmware rate is REF_HZ-relative.
    parameter int unsigned CLK_HZ = 70_588_235,
    parameter int unsigned REF_HZ = 1_843_200,
    // Byte presented on int_vec_o for Z80 IM2 / Z180 IM2 acknowledge (the real
    // 16550 has no vector register; IM2 boards supply it externally).
    parameter logic [7:0] IRQ_VECTOR = 8'hFF,
    // Reset value of the baud divisor {DLM,DLL}. 0 (default, the real 16550) leaves
    // the baud clock disabled until firmware programs DLL/DLM. A board that taps the
    // baud ticks before firmware runs (a second UART slaving its line rate to this
    // one -- see baud_*_tick_o) sets it so the ticks are live out of reset
    // (e.g. 1 -> 115200 at REF_HZ 1.8432 MHz). Firmware still reprograms it.
    parameter logic [15:0] RESET_DIVISOR = 16'h0000
) (
    input wire logic clk_i,
    input wire logic nrst_i, // synchronous, active-low

    // ---- CPU register interface (the SoC does the port decode) ---------------
    input  wire logic       cs_i,    // 1 = this access targets the UART window
    input  wire logic [2:0] addr_i,  // register select A2..A0
    input  wire logic       rd_i,    // one-clk read strobe (pulse once per IORQ read)
    input  wire logic       wr_i,    // one-clk write strobe (pulse once per IORQ write)
    input  wire logic [7:0] din_i,   // write data
    output logic      [7:0] dout_o,  // read data (combinational; valid while cs_i & rd_i)

    // ---- Interrupt ----
    output logic       int_o,     // level INT, OUT2-gated (active high)
    output logic [7:0] int_vec_o, // IM2 / Z180 vector byte (= IRQ_VECTOR)

    // ---- Serial pins ----
    output logic      txd_o,
    input  wire logic rxd_i,

    // ---- Modem control/status pins (active-low; tie high if unused) ----
    input  wire logic cts_n_i,
    input  wire logic dsr_n_i,
    input  wire logic dcd_n_i,
    input  wire logic ri_n_i,
    output logic      rts_n_o,
    output logic      dtr_n_o,

    // ---- Baud generator ticks (for an external consumer slaving its line rate to
    //      this UART, e.g. a second UART on the same console). Free-running at
    //      the firmware-programmed baud with matching CLK_HZ/REF_HZ parameters. ----
    output logic baud_bit_tick_o,    // one pulse per bit period
    output logic baud_samp16_tick_o  // 16 pulses per bit period (RX oversample)
);

  // ── Variant capabilities ────────────────────────────────────────────────
  localparam bit HasFifo = (VARIANT == 3);  // 16550A: working FIFOs
  localparam bit FifoBuggy = (VARIANT == 2);  // 16550: reports FIFO, doesn't buffer
  localparam bit HasScr = (VARIANT != 0);  // 16450+ have the scratch register
  localparam int unsigned FifoDepth = 16;

  // ── Register offsets ──────────────────────────────────────────────────────
  localparam logic [2:0] RegRbrThrDll = 3'd0;
  localparam logic [2:0] RegIerDlm = 3'd1;
  localparam logic [2:0] RegIirFcr = 3'd2;
  localparam logic [2:0] RegLcr = 3'd3;
  localparam logic [2:0] RegMcr = 3'd4;
  localparam logic [2:0] RegLsr = 3'd5;
  localparam logic [2:0] RegMsr = 3'd6;
  localparam logic [2:0] RegScr = 3'd7;

  // ── IER / IIR enables and codes ────────────────────────────────────────────
  localparam logic [3:0] IerErbfi = 4'h1;  // RX data available
  localparam logic [3:0] IerEtbei = 4'h2;  // THR empty
  localparam logic [3:0] IerElsi = 4'h4;  // line status
  localparam logic [3:0] IerEdssi = 4'h8;  // modem status

  localparam logic [3:0] IirNoInt = 4'h1;
  localparam logic [3:0] IirModem = 4'h0;
  localparam logic [3:0] IirThre = 4'h2;
  localparam logic [3:0] IirRda = 4'h4;
  localparam logic [3:0] IirRls = 4'h6;
  localparam logic [3:0] IirTimeout = 4'hC;

  // uart_rx_core err_o / rxf_err_q bit positions ({BI, FE, PE}).
  localparam int unsigned ErrPe = 0;  // parity error
  localparam int unsigned ErrFe = 1;  // framing error
  localparam int unsigned ErrBi = 2;  // break

  // ── Programmable register state ─────────────────────────────────────────
  logic [7:0] dll_q, dlm_q;
  logic [3:0] ier_q;
  logic [7:0] lcr_q;
  logic [4:0] mcr_q;
  logic [7:0] scr_q;

  logic lsr_oe_q, lsr_pe_q, lsr_fe_q, lsr_bi_q;  // sticky, cleared on LSR read

  logic       fifo_en_q;
  logic [3:0] rx_trig_q;  // 1, 4, 8, or 14

  logic       thre_pend_q;  // THRE level-interrupt pending
  logic       thre_q;  // THRE last clock, for its rising edge
  logic       thre_two_q;  // two bytes queued at once since THRE last rose
  logic       thre_fcr_q;  // FCR0 changed since THRE last rose
  logic       thre_dly_q;  // FIFO mode: the THRE interrupt is being held back
  logic [3:0] thre_dly_cnt_q;  // bit ticks left in the hold-back

  logic msr_cts_q, msr_dsr_q, msr_dcd_q, msr_ri_q;
  logic msr_dcts_q, msr_ddsr_q, msr_ddcd_q, msr_teri_q;
  // Counts the first three clocks after reset, while the pin synchronizers
  // fill; the delta bits latch only once it saturates, so a modem line held
  // asserted through reset is not reported as a change.
  logic [1:0] msr_arm_q;

  // Modem input pin synchronizers (active-low pins -> active-high, 2-FF).
  logic cts_m, cts_s, dsr_m, dsr_s, dcd_m, dcd_s, ri_m, ri_s;

  // LCR / MCR field accessors.
  wire  [ 1:0] lcr_wls = lcr_q[1:0];
  wire         lcr_stb = lcr_q[2];
  wire         lcr_pen = lcr_q[3];
  wire         lcr_eps = lcr_q[4];
  wire         lcr_sp = lcr_q[5];
  wire         lcr_brk = lcr_q[6];
  wire         dlab = lcr_q[7];
  wire         mcr_dtr = mcr_q[0];
  wire         mcr_rts = mcr_q[1];
  wire         mcr_out1 = mcr_q[2];
  wire         mcr_out2 = mcr_q[3];
  wire         mcr_loop = mcr_q[4];

  // ── Non-FIFO holding registers ────────────────────────────────────────────
  logic [ 7:0] rbr_q;
  logic        rbr_valid_q;
  logic [ 7:0] thr_q;
  logic        thr_valid_q;

  // ── RX FIFO (16550A): data + per-byte {BI,FE,PE} ──────────────────────────
  logic [ 7:0] rxf_data_q                         [FifoDepth];
  logic [ 2:0] rxf_err_q                          [FifoDepth];
  logic [ 4:0] rxf_count_q;
  logic [ 3:0] rxf_head_q;
  logic [ 3:0] rxf_tail_q;
  // The head entry's {BI,FE,PE} have been folded into the LSR. The data sheet
  // reveals a character's errors when it reaches the top of the FIFO, ahead of
  // the RBR read that takes it.
  logic        rxf_shown_q;
  // Entries carrying an error, and FIFOE (LSR D7): set while any is in the
  // FIFO, cleared by an LSR read once none is.
  logic [ 4:0] rxf_errcnt_q;
  logic        fifoe_q;

  // ── TX FIFO (16550A) ──────────────────────────────────────────────────────
  logic [ 7:0] txf_data_q                         [FifoDepth];
  logic [ 4:0] txf_count_q;
  logic [ 3:0] txf_head_q;
  logic [ 3:0] txf_tail_q;

  wire         fifo_active = HasFifo && fifo_en_q;

  // ── Character-timeout tracking (FIFO mode) ────────────────────────────────
  // 6 bits: the counter saturates at to_threshold, which maxes at 48 (4 x the
  // 12-bit-frame maximum).
  logic [ 5:0] to_cnt_q;
  logic        to_fired_q;

  // ── PHY wiring ────────────────────────────────────────────────────────────
  logic        baud_tick;
  logic        tx_we;
  logic [ 7:0] tx_byte;
  logic        tx_ready;
  logic        tx_idle;
  logic        txd_int;
  logic        rx_stb;
  logic [ 7:0] rx_byte;
  logic [ 2:0] rx_err;
  logic        rxd_int;

  // ════════════════════════════════════════════════════════════════════════
  //  Baud generator -- fractional division with a matching CLK_HZ parameter
  //  (uart_baud_dds); baud = REF_HZ / (16 * divisor).
  // ════════════════════════════════════════════════════════════════════════
  wire  [15:0] divisor = {dlm_q, dll_q};
  uart_baud_dds #(
      .CLK_HZ(CLK_HZ),
      .REF_HZ(REF_HZ)
  ) u_baud (
      .clk_i        (clk_i),
      .nrst_i       (nrst_i),
      .divisor_i    (divisor),
      .resync_i     (1'b0),                // the TX FSM starts a frame on a bit tick
      .samp16_tick_o(baud_samp16_tick_o),  // exposed for an external baud consumer
      .bit_tick_o   (baud_tick)
  );
  assign baud_bit_tick_o = baud_tick;

  // ── Serial PHY + loopback mux ─────────────────────────────────────────────
  assign rxd_int         = mcr_loop ? txd_int : rxd_i;
  assign txd_o           = mcr_loop ? 1'b1 : (lcr_brk ? 1'b0 : txd_int);

  // No holding buffer: tx_ready is the clock the shifter can load, and a byte
  // offered then starts its frame on that tick.
  uart_tx_core #(
      .HOLD(1'b0)
  ) u_tx (
      .clk_i         (clk_i),
      .nrst_i        (nrst_i),
      .baud_tick_i   (baud_tick),
      .data_i        (tx_byte),
      .we_i          (tx_we),
      .word_len_i    (lcr_wls),
      .parity_en_i   (lcr_pen),
      .parity_even_i (lcr_eps),
      .parity_stick_i(lcr_sp),
      .stop2_i       (lcr_stb),
      .break_i       (lcr_brk),
      .ready_o       (tx_ready),
      .tx_idle_o     (tx_idle),
      .txd_o         (txd_int)
  );

  uart_rx_core u_rx (
      .clk_i         (clk_i),
      .nrst_i        (nrst_i),
      .samp16_tick_i (baud_samp16_tick_o),
      .rxd_i         (rxd_int),
      .word_len_i    (lcr_wls),
      .parity_en_i   (lcr_pen),
      .parity_even_i (lcr_eps),
      .parity_stick_i(lcr_sp),
      .data_o        (rx_byte),
      .err_o         (rx_err),
      .stb_o         (rx_stb)
  );

  // Frame bit count (start + data + parity + stop): the 4-char timeout and the
  // THRE interrupt hold-back.
  wire [3:0] frame_bits = 4'd1 + (4'd5 + {2'b00, lcr_wls}) + (lcr_pen ? 4'd1 : 4'd0)
      + (lcr_stb ? 4'd2 : 4'd1);
  wire [5:0] to_threshold = {frame_bits, 2'h0};  // 4 * frame_bits bit-periods

  // ════════════════════════════════════════════════════════════════════════
  //  Read data path (combinational; side effects in the always_ff below).
  // ════════════════════════════════════════════════════════════════════════
  wire rx_data_ready = fifo_active ? (rxf_count_q != 5'd0) : rbr_valid_q;
  wire [7:0] rbr_head = fifo_active ? rxf_data_q[rxf_head_q] : rbr_q;
  wire tx_empty = fifo_active ? (txf_count_q == 5'd0) : ~thr_valid_q;

  logic [7:0] lsr_val;
  always_comb begin
    lsr_val    = 8'h00;
    lsr_val[0] = rx_data_ready;  // DR
    lsr_val[1] = lsr_oe_q;  // OE
    lsr_val[2] = lsr_pe_q;  // PE
    lsr_val[3] = lsr_fe_q;  // FE
    lsr_val[4] = lsr_bi_q;  // BI
    lsr_val[5] = tx_empty;  // THRE
    lsr_val[6] = tx_empty & tx_idle;  // TEMT
    lsr_val[7] = fifoe_q;  // FIFOE (16550A)
  end

  logic [7:0] msr_val;
  always_comb begin
    msr_val = 8'h00;
    if (mcr_loop) begin
      msr_val[4] = mcr_rts;  // CTS <- RTS
      msr_val[5] = mcr_dtr;  // DSR <- DTR
      msr_val[6] = mcr_out1;  // RI  <- OUT1
      msr_val[7] = mcr_out2;  // DCD <- OUT2
    end else begin
      msr_val[4] = msr_cts_q;
      msr_val[5] = msr_dsr_q;
      msr_val[6] = msr_ri_q;
      msr_val[7] = msr_dcd_q;
    end
    msr_val[0] = msr_dcts_q;
    msr_val[1] = msr_ddsr_q;
    msr_val[2] = msr_teri_q;
    msr_val[3] = msr_ddcd_q;
  end

  wire [1:0] fifo_status_bits = (fifo_en_q && HasFifo) ? 2'b11 :
      ((fifo_en_q && FifoBuggy) ? 2'b10 : 2'b00);
  wire modem_delta = msr_dcts_q || msr_ddsr_q || msr_ddcd_q || msr_teri_q;
  wire line_status = lsr_oe_q || lsr_pe_q || lsr_fe_q || lsr_bi_q;
  wire rda_active = fifo_active ? (rxf_count_q >= {1'b0, rx_trig_q}) : rbr_valid_q;
  wire to_active = fifo_active && (rxf_count_q != 5'd0) && to_fired_q;

  logic [3:0] iir_code;
  always_comb begin
    if ((ier_q & IerElsi) != 4'd0 && line_status) iir_code = IirRls;
    else if ((ier_q & IerErbfi) != 4'd0 && rda_active) iir_code = IirRda;
    else if ((ier_q & IerErbfi) != 4'd0 && to_active) iir_code = IirTimeout;
    else if ((ier_q & IerEtbei) != 4'd0 && thre_pend_q) iir_code = IirThre;
    else if ((ier_q & IerEdssi) != 4'd0 && modem_delta) iir_code = IirModem;
    else iir_code = IirNoInt;
  end
  wire [7:0] iir_val = {fifo_status_bits, 2'b00, iir_code};

  always_comb begin
    unique case (addr_i)
      RegRbrThrDll: dout_o = dlab ? dll_q : rbr_head;
      RegIerDlm:    dout_o = dlab ? dlm_q : {4'h0, ier_q};
      RegIirFcr:    dout_o = iir_val;
      RegLcr:       dout_o = lcr_q;
      RegMcr:       dout_o = {3'h0, mcr_q};
      RegLsr:       dout_o = lsr_val;
      RegMsr:       dout_o = msr_val;
      default:      dout_o = HasScr ? scr_q : 8'h00;  // RegScr, the last of the eight
    endcase
  end

  assign int_o     = (mcr_loop || mcr_out2) && (iir_code != IirNoInt);
  assign int_vec_o = IRQ_VECTOR;
  // Loopback forces the modem-control outputs inactive (high).
  assign rts_n_o   = ~(mcr_rts && !mcr_loop);
  assign dtr_n_o   = ~(mcr_dtr && !mcr_loop);

  // ════════════════════════════════════════════════════════════════════════
  //  Access decode (single-cycle strobes the SoC pulses once per IORQ).
  // ════════════════════════════════════════════════════════════════════════
  wire wr = cs_i && wr_i;
  wire rd = cs_i && rd_i;
  wire wr_rbr_thr = wr && (addr_i == RegRbrThrDll) && !dlab;
  wire wr_dll = wr && (addr_i == RegRbrThrDll) && dlab;
  wire wr_ier = wr && (addr_i == RegIerDlm) && !dlab;
  wire wr_dlm = wr && (addr_i == RegIerDlm) && dlab;
  wire wr_fcr = wr && (addr_i == RegIirFcr);
  wire wr_lcr = wr && (addr_i == RegLcr);
  wire wr_mcr = wr && (addr_i == RegMcr);
  wire wr_scr = wr && (addr_i == RegScr);
  wire rd_rbr = rd && (addr_i == RegRbrThrDll) && !dlab;
  wire rd_iir = rd && (addr_i == RegIirFcr);
  wire rd_lsr = rd && (addr_i == RegLsr);
  wire rd_msr = rd && (addr_i == RegMsr);

  // ── FIFO push/pop decisions (computed once so simultaneity is conflict-safe) ─
  // FCR1 / FCR2 clear a FIFO, and so does any change of FCR0 (the data sheet:
  // switching between FIFO and 16450 mode clears both FIFOs). The FCR1/FCR2
  // bits take effect only with FCR0 = 1 in the same write.
  wire fcr_mode_chg = wr_fcr && HasFifo && (din_i[0] != fifo_en_q);
  wire rx_clr = wr_fcr && HasFifo && ((din_i[0] && din_i[1]) || fcr_mode_chg);
  wire tx_clr = wr_fcr && HasFifo && ((din_i[0] && din_i[2]) || fcr_mode_chg);

  // The shifter takes the waiting byte (the TX FIFO head, or the THR) in the
  // clock it can load, unless a clear in that clock discards it.
  wire tx_pop = fifo_active && tx_ready && (txf_count_q != 5'd0) && !tx_clr;
  wire thr_move = !fifo_active && tx_ready && thr_valid_q && !tx_clr;
  assign tx_we   = tx_pop || thr_move;
  assign tx_byte = fifo_active ? txf_data_q[txf_head_q] : thr_q;
  // A write to a full TX FIFO is dropped, unless the shifter takes a byte in
  // that clock. A 16450-mode THR write replaces any waiting byte.
  wire tx_full = (txf_count_q == FifoDepth[4:0]);
  wire tx_push = fifo_active && wr_rbr_thr && (!tx_full || tx_pop);
  wire thr_load = !fifo_active && wr_rbr_thr;

  wire rx_full = (rxf_count_q == FifoDepth[4:0]);
  wire rx_pop = fifo_active && rd_rbr && (rxf_count_q != 5'd0);
  // A byte completing into a full FIFO in the clock of an RBR read takes the
  // place that read frees.
  wire rx_push = fifo_active && rx_stb && (!rx_full || rx_pop);

  // Errored entries in the RX FIFO, and FIFOE. An error landing in the clock
  // of an LSR read survives it.
  wire rx_push_err = rx_push && (rx_err != 3'd0);
  wire rx_pop_err = rx_pop && (rxf_err_q[rxf_head_q] != 3'd0);
  wire [4:0] errcnt_next = rx_clr ? 5'd0 : rxf_errcnt_q + {4'd0, rx_push_err} - {4'd0, rx_pop_err};
  wire fifoe_next = (errcnt_next != 5'd0) || (fifoe_q && !rd_lsr && !fcr_mode_chg);

  wire thre_rise = tx_empty && !thre_q;

  integer i;
  always_ff @(posedge clk_i) begin
    if (!nrst_i) begin
      dll_q          <= RESET_DIVISOR[7:0];
      dlm_q          <= RESET_DIVISOR[15:8];
      ier_q          <= 4'h0;
      lcr_q          <= 8'h00;
      mcr_q          <= 5'h00;
      scr_q          <= 8'h00;
      lsr_oe_q       <= 1'b0;
      lsr_pe_q       <= 1'b0;
      lsr_fe_q       <= 1'b0;
      lsr_bi_q       <= 1'b0;
      fifo_en_q      <= 1'b0;
      rx_trig_q      <= 4'd1;
      thre_pend_q    <= 1'b1;  // THRE set at reset
      thre_q         <= 1'b1;
      thre_two_q     <= 1'b0;
      thre_fcr_q     <= 1'b0;
      thre_dly_q     <= 1'b0;
      thre_dly_cnt_q <= 4'd0;
      rxf_errcnt_q   <= 5'd0;
      fifoe_q        <= 1'b0;
      rbr_q          <= 8'h00;
      rbr_valid_q    <= 1'b0;
      thr_q          <= 8'h00;
      thr_valid_q    <= 1'b0;
      rxf_count_q    <= 5'd0;
      rxf_head_q     <= 4'd0;
      rxf_tail_q     <= 4'd0;
      rxf_shown_q    <= 1'b0;
      txf_count_q    <= 5'd0;
      txf_head_q     <= 4'd0;
      txf_tail_q     <= 4'd0;
      to_cnt_q       <= 6'd0;
      to_fired_q     <= 1'b0;
      msr_cts_q      <= 1'b0;
      msr_dsr_q      <= 1'b0;
      msr_dcd_q      <= 1'b0;
      msr_ri_q       <= 1'b0;
      msr_dcts_q     <= 1'b0;
      msr_ddsr_q     <= 1'b0;
      msr_ddcd_q     <= 1'b0;
      msr_teri_q     <= 1'b0;
      msr_arm_q      <= 2'd0;
      cts_m          <= 1'b0;
      cts_s          <= 1'b0;
      dsr_m          <= 1'b0;
      dsr_s          <= 1'b0;
      dcd_m          <= 1'b0;
      dcd_s          <= 1'b0;
      ri_m           <= 1'b0;
      ri_s           <= 1'b0;
      for (i = 0; i < FifoDepth; i = i + 1) begin
        rxf_data_q[i] <= 8'h00;
        rxf_err_q[i]  <= 3'h0;
        txf_data_q[i] <= 8'h00;
      end
    end else begin
      // ── Register writes ──────────────────────────────────────────────
      if (wr_dll) dll_q <= din_i;
      if (wr_dlm) dlm_q <= din_i;
      if (wr_lcr) lcr_q <= din_i;
      if (wr_scr && HasScr) scr_q <= din_i;
      if (wr_mcr) mcr_q <= din_i[4:0];
      if (wr_ier) ier_q <= din_i[3:0];
      if (wr_fcr) begin
        if (HasFifo) begin
          fifo_en_q <= din_i[0];
          if (din_i[0]) begin
            case (din_i[7:6])
              2'b00:   rx_trig_q <= 4'd1;
              2'b01:   rx_trig_q <= 4'd4;
              2'b10:   rx_trig_q <= 4'd8;
              default: rx_trig_q <= 4'd14;
            endcase
          end
        end else if (FifoBuggy) begin
          fifo_en_q <= din_i[0];  // reports enabled; fifo_active stays 0 (never buffers)
        end
      end

      // ── TX: the shifter takes the waiting byte; THR write / FIFO push ─
      // The THR write comes after the move, so a byte written in the clock
      // the shifter takes the old one waits in its place.
      if (thr_move) thr_valid_q <= 1'b0;
      if (thr_load) begin
        thr_q       <= din_i;
        thr_valid_q <= 1'b1;
      end
      if (tx_pop) txf_head_q <= txf_head_q + 4'd1;
      if (tx_push) begin
        txf_data_q[txf_tail_q] <= din_i;
        txf_tail_q             <= txf_tail_q + 4'd1;
      end
      if (fifo_active) begin
        txf_count_q <= txf_count_q + (tx_push ? 5'd1 : 5'd0) - (tx_pop ? 5'd1 : 5'd0);
      end

      // ── LSR read: clear the sticky line-status bits. It comes before the
      //    receive logic below so a flag raised in the same clock survives.
      if (rd_lsr) begin
        lsr_oe_q <= 1'b0;
        lsr_pe_q <= 1'b0;
        lsr_fe_q <= 1'b0;
        lsr_bi_q <= 1'b0;
      end

      // ── RX capture + RBR read (push/pop combined) ────────────────────
      if (rx_stb) begin
        to_cnt_q   <= 6'd0;
        to_fired_q <= 1'b0;
      end
      if (rx_push) begin
        rxf_data_q[rxf_tail_q] <= rx_byte;
        rxf_err_q[rxf_tail_q]  <= rx_err;
        rxf_tail_q             <= rxf_tail_q + 4'd1;
      end else if (fifo_active && rx_stb) begin
        lsr_oe_q <= 1'b1;  // RX FIFO full and not read -> overrun
      end
      rxf_errcnt_q <= errcnt_next;
      fifoe_q      <= fifoe_next;

      if (rd_rbr) begin
        to_cnt_q   <= 6'd0;
        to_fired_q <= 1'b0;
      end
      if (rx_pop) rxf_head_q <= rxf_head_q + 4'd1;

      if (fifo_active) begin
        rxf_count_q <= rxf_count_q + (rx_push ? 5'd1 : 5'd0) - (rx_pop ? 5'd1 : 5'd0);
        // The head's errors reach the LSR once, the clock after it becomes the
        // head; a pop makes the next entry the head, still to be shown.
        if (rxf_count_q != 5'd0 && !rxf_shown_q) begin
          if (rxf_err_q[rxf_head_q][ErrPe]) lsr_pe_q <= 1'b1;
          if (rxf_err_q[rxf_head_q][ErrFe]) lsr_fe_q <= 1'b1;
          if (rxf_err_q[rxf_head_q][ErrBi]) lsr_bi_q <= 1'b1;
        end
        if (rx_pop) rxf_shown_q <= 1'b0;
        else if (rxf_count_q != 5'd0) rxf_shown_q <= 1'b1;
      end

      // Non-FIFO RX/RBR (when FIFO inactive). The error bits are sticky until
      // an LSR read, like OE; a later clean character does not clear them.
      if (!fifo_active) begin
        if (rx_stb) begin
          // A byte landing in the clock the RBR is read replaces it, no overrun.
          if (rbr_valid_q && !rd_rbr) lsr_oe_q <= 1'b1;
          rbr_q       <= rx_byte;
          rbr_valid_q <= 1'b1;
          if (rx_err[ErrPe]) lsr_pe_q <= 1'b1;
          if (rx_err[ErrFe]) lsr_fe_q <= 1'b1;
          if (rx_err[ErrBi]) lsr_bi_q <= 1'b1;
        end else if (rd_rbr) begin
          rbr_valid_q <= 1'b0;
        end
      end

      // ── THRE interrupt ───────────────────────────────────────────────
      // THRE's rise sets it: at once in 16450 mode; in FIFO mode at once if two
      // bytes were queued together, or FCR0 changed, since THRE last rose, and
      // otherwise a character time less the last stop bit later (frame_bits - 1
      // bit ticks), which a THR write cancels.
      thre_q <= tx_empty;
      if (txf_count_q >= 5'd2) thre_two_q <= 1'b1;
      if (thre_rise) begin
        thre_two_q <= 1'b0;
        thre_fcr_q <= 1'b0;
        if (!fifo_active || thre_two_q || thre_fcr_q) begin
          thre_pend_q <= 1'b1;
        end else begin
          thre_dly_q     <= 1'b1;
          thre_dly_cnt_q <= frame_bits - 4'd1;
        end
      end
      if (thre_dly_q && baud_tick) begin
        if (thre_dly_cnt_q == 4'd1) begin
          thre_dly_q  <= 1'b0;
          thre_pend_q <= 1'b1;
        end else begin
          thre_dly_cnt_q <= thre_dly_cnt_q - 4'd1;
        end
      end
      // An ETBEI write with THRE set raises it at once. A THR write clears it,
      // and so does an IIR read that reports it.
      if (wr_ier && (din_i[3:0] & IerEtbei) != 4'd0 && tx_empty) begin
        thre_pend_q <= 1'b1;
        thre_dly_q  <= 1'b0;
      end
      if (wr_rbr_thr) begin
        thre_pend_q <= 1'b0;
        thre_dly_q  <= 1'b0;
      end
      if (rd_iir && iir_code == IirThre) thre_pend_q <= 1'b0;
      // A TX FIFO clear (an FCR0 change among them) empties every waiting
      // byte, so it sets THRE and raises the interrupt.
      if (tx_clr) begin
        thre_dly_q  <= 1'b0;
        thre_pend_q <= 1'b1;
        thre_q      <= 1'b1;  // THRE's rise from the clear is this one
      end
      if (fcr_mode_chg) thre_fcr_q <= 1'b1;

      // ── MSR read clears the deltas ───────────────────────────────────
      if (rd_msr) begin
        msr_dcts_q <= 1'b0;
        msr_ddsr_q <= 1'b0;
        msr_ddcd_q <= 1'b0;
        msr_teri_q <= 1'b0;
      end

      // ── Character timeout (FIFO mode): 4 char-times of RX inactivity ──
      if (fifo_active && (rxf_count_q != 5'd0) && !rx_stb && !rd_rbr && baud_tick) begin
        if (to_cnt_q >= to_threshold) to_fired_q <= 1'b1;
        else to_cnt_q <= to_cnt_q + 6'd1;
      end

      // ── FIFO clears last, so they win over a same-clock push, pop or tick ─
      if (rx_clr) begin
        rxf_count_q <= 5'd0;
        rxf_head_q  <= 4'd0;
        rxf_tail_q  <= 4'd0;
        rxf_shown_q <= 1'b0;
        to_cnt_q    <= 6'd0;
        to_fired_q  <= 1'b0;
      end
      if (tx_clr) begin
        txf_count_q <= 5'd0;
        txf_head_q  <= 4'd0;
        txf_tail_q  <= 4'd0;
      end
      if (fcr_mode_chg) begin  // the 16450-mode holding registers are FIFO slot 0
        rbr_valid_q <= 1'b0;
        thr_valid_q <= 1'b0;
      end

      // ── Modem pins: synchronize + latch deltas ───────────────────────
      cts_m <= ~cts_n_i;
      cts_s <= cts_m;
      dsr_m <= ~dsr_n_i;
      dsr_s <= dsr_m;
      dcd_m <= ~dcd_n_i;
      dcd_s <= dcd_m;
      ri_m  <= ~ri_n_i;
      ri_s  <= ri_m;
      if (msr_arm_q != 2'd3) msr_arm_q <= msr_arm_q + 2'd1;
      if (!mcr_loop) begin
        if (msr_arm_q == 2'd3) begin
          if (cts_s != msr_cts_q) msr_dcts_q <= 1'b1;
          if (dsr_s != msr_dsr_q) msr_ddsr_q <= 1'b1;
          if (dcd_s != msr_dcd_q) msr_ddcd_q <= 1'b1;
          if (msr_ri_q && !ri_s) msr_teri_q <= 1'b1;
        end
        msr_cts_q <= cts_s;
        msr_dsr_q <= dsr_s;
        msr_dcd_q <= dcd_s;
        msr_ri_q  <= ri_s;
      end else begin
        if (mcr_rts != msr_cts_q) msr_dcts_q <= 1'b1;
        if (mcr_dtr != msr_dsr_q) msr_ddsr_q <= 1'b1;
        if (mcr_out2 != msr_dcd_q) msr_ddcd_q <= 1'b1;
        if (msr_ri_q && !mcr_out1) msr_teri_q <= 1'b1;
        msr_cts_q <= mcr_rts;
        msr_dsr_q <= mcr_dtr;
        msr_dcd_q <= mcr_out2;
        msr_ri_q  <= mcr_out1;
      end
    end
  end

endmodule : uart16550

`default_nettype wire
