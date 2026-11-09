// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// Integer clock-phase arithmetic predicts DDS reference events. A single
// reference-event count models sample spacing; a sample-token count models
// rephasing. Floor/ceiling interval checks cover fractional core/reference ratios.
module baud_timing_formal #(
    parameter int unsigned CLOCK = 13,
    parameter int unsigned REFERENCE = 5,
    parameter int unsigned DIVISOR = 3
) (
    input wire logic clk,
    input wire logic nrst,
    input wire logic resync
);
  wire sample_tick, bit_tick;
  uart_baud_dds #(
      .CLK_HZ(CLOCK),
      .REF_HZ(REFERENCE)
  ) dut (
      .clk_i        (clk),
      .nrst_i       (nrst),
      .divisor_i    (16'(DIVISOR)),
      .resync_i     (resync),
      .samp16_tick_o(sample_tick),
      .bit_tick_o   (bit_tick)
  );
  localparam int unsigned CountModulus = DIVISOR == 0 ? 1 : 16 * DIVISOR;
  localparam int unsigned SampleFloor = CLOCK * DIVISOR / REFERENCE;
  localparam int unsigned SampleCeil = (CLOCK * DIVISOR + REFERENCE - 1) / REFERENCE;
  localparam int unsigned BitFloor = 16 * CLOCK * DIVISOR / REFERENCE;
  localparam int unsigned BitCeil = (16 * CLOCK * DIVISOR + REFERENCE - 1) / REFERENCE;
  logic [$clog2(CLOCK)-1:0] cycle_phase;
  logic [$clog2(CountModulus+1)-1:0] reference_events;
  logic [4:0] sample_tokens;
  logic ref_expected, sample_expected, bit_expected;
  logic [$clog2(SampleCeil+4)-1:0] sample_age;
  logic [$clog2(BitCeil+4)-1:0] bit_age;
  logic saw_sample, saw_bit, rephased;
  logic [31:0] last_sample_phase, last_bit_phase;
  wire [31:0] reference_phase = (int'(cycle_phase) * REFERENCE) % CLOCK;
  wire [31:0] sample_phase = DIVISOR == 0 ? 0 : reference_phase +
      CLOCK * (int'(ref_expected) + int'(reference_events) % DIVISOR);
  wire [31:0] bit_phase = sample_phase + CLOCK * DIVISOR *
      (int'(sample_tokens) + int'(sample_expected));
  logic past_valid = 0;
  always @(posedge clk) past_valid <= 1;
  always_comb if (!past_valid) assume (!nrst);
  always @(posedge clk) begin
    if (!nrst) begin
      cycle_phase       <= 0;
      reference_events  <= 0;
      sample_tokens     <= 0;
      ref_expected      <= 0;
      sample_expected   <= 0;
      bit_expected      <= 0;
      saw_sample        <= 0;
      saw_bit           <= 0;
      rephased          <= 0;
      sample_age        <= 0;
      bit_age           <= 0;
      last_sample_phase <= 0;
      last_bit_phase    <= 0;
    end else begin
      cycle_phase     <= cycle_phase == CLOCK - 1 ? 0 : cycle_phase + 1'b1;
      ref_expected    <= ((int'(cycle_phase) * REFERENCE) % CLOCK) + REFERENCE >= CLOCK;
      sample_expected <= 0;
      bit_expected    <= 0;
      if (DIVISOR != 0 && ref_expected) begin
        reference_events <= reference_events == CountModulus - 1 ? 0 : reference_events + 1'b1;
        sample_expected  <= int'(reference_events) % DIVISOR == DIVISOR - 1;
      end
      if (resync) begin
        sample_tokens <= 0;
        rephased      <= 1;
        saw_bit       <= 0;
        bit_age       <= 0;
      end else begin
        if (sample_expected) begin
          if (sample_tokens == 15) begin
            sample_tokens <= 0;
            bit_expected  <= 1;
          end else sample_tokens <= sample_tokens + 1'b1;
        end
        if (bit_tick) begin
          last_bit_phase <= bit_phase;
          if (saw_bit) assert (int'(bit_age) + 1 >= BitFloor && int'(bit_age) + 1 <= BitCeil);
          saw_bit <= 1;
          bit_age <= 0;
        end else if (saw_bit) begin
          assert (bit_age < BitCeil);
          bit_age <= bit_age + 1'b1;
        end
      end
      if (sample_tick) begin
        last_sample_phase <= sample_phase;
        if (saw_sample)
          assert (int'(sample_age) + 1 >= SampleFloor && int'(sample_age) + 1 <= SampleCeil);
        saw_sample <= 1;
        sample_age <= 0;
      end else if (saw_sample) begin
        assert (sample_age < SampleCeil);
        sample_age <= sample_age + 1'b1;
      end
    end
    if (past_valid) begin
      assert (cycle_phase < CLOCK);
      assert (dut.ref_acc_q == (int'(cycle_phase) * REFERENCE) % CLOCK);
      assert (dut.ref_tick == ref_expected);
      if ($past(nrst)) assert (ref_expected == (reference_phase < REFERENCE));
      assert (sample_tick == sample_expected);
      assert (bit_tick == bit_expected);
      assert (reference_events < CountModulus);
      assert (sample_tokens < 16);
      assert (dut.bit_cnt_q == sample_tokens[3:0]);
      if (DIVISOR == 0) begin
        assert (!sample_tick && !bit_tick);
        cover (nrst && cycle_phase == CLOCK - 1);
      end else begin
        assert (dut.samp_cnt_q == int'(reference_events) % DIVISOR);
        // Conservation links elapsed core clocks to the fractional phase and
        // outstanding reference/sample tokens, without modular wrap loss.
        if (saw_sample && nrst)
          assert (sample_phase + (sample_tick ? CLOCK * DIVISOR : 0) ==
                  last_sample_phase + (int'(sample_age) + 1) * REFERENCE);
        if (saw_bit && nrst && !resync)
          assert (bit_phase + (bit_tick ? 16 * CLOCK * DIVISOR : 0) ==
                  last_bit_phase + (int'(bit_age) + 1) * REFERENCE);
        if (sample_tick) assert (sample_phase >= REFERENCE && sample_phase < 2 * REFERENCE);
        if (bit_tick) assert (bit_phase >= 2 * REFERENCE && bit_phase < 3 * REFERENCE);
        if (saw_sample)
          assert (last_sample_phase >= REFERENCE && last_sample_phase < 2 * REFERENCE);
        if (saw_bit) assert (last_bit_phase >= 2 * REFERENCE && last_bit_phase < 3 * REFERENCE);
        cover (nrst && bit_tick && saw_bit);
        cover (nrst && rephased && bit_tick);
`ifdef FORMAL_NEGATIVE
        assert (!sample_tick);
`endif
      end
    end
  end
endmodule
