// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
// End-to-end receive contract: a frame every 256 clocks, sixteen core/sample
// clocks per wire bit, and arbitrary line values in the two boundary clocks
// (phases 15 and 0). Configuration and payload are symbolic and held stable.
// This finite jitter envelope is a digital model, not a metastability proof.
module rx_wire_formal (
    input wire logic       clk,
    nrst,
    boundary_noise,
    input wire logic [7:0] frame_byte,
    input wire logic [1:0] length,
    input wire logic       parity_enable,
    parity_even,
    parity_stick,
    input wire logic       bad_parity,
    bad_stop
);
  logic past_valid = 0;
  logic [7:0] phase;
  logic delivered;
  logic [7:0] start_offset;
  wire [3:0] data_bits = 5 + {2'b0, length};
  wire [3:0] stop_slot = data_bits + 1'b1 + 4'(parity_enable);
  wire [8:0] frame_end = 32 + 16 * (stop_slot + 1);
  wire [8:0] delivery_phase = {1'b0, start_offset} + 8 + 16 * stop_slot;
  wire [7:0] data_mask = 8'hFF >> (8 - data_bits);
  wire [14:0] frame_config = {
    frame_byte, length, parity_enable, parity_even, parity_stick, bad_parity, bad_stop
  };
  logic parity_value;
  always_comb begin
    parity_value = !parity_even;
    if (!parity_stick) for (int i = 0; i < 8; i++) if (i < data_bits) parity_value ^= frame_byte[i];
  end
  wire  wire_parity = parity_value ^ bad_parity;
  logic line_in;
  always_comb begin
    line_in = 1;
    if (phase >= 32 && {1'b0, phase} < frame_end) begin
      if (phase[3:0] == 0 || phase[3:0] == 15) line_in = boundary_noise;
      else if ((phase - 32) / 16 == 0) line_in = 0;
      else if ((phase - 32) / 16 <= data_bits) line_in = frame_byte[(phase-32)/16-1];
      else if (parity_enable && (phase - 32) / 16 < stop_slot) line_in = wire_parity;
      else line_in = !bad_stop;
    end
  end
  rx_frame_formal receiver (
      .clk          (clk),
      .nrst         (nrst),
      .tick         (1'b1),
      .line_in      (line_in),
      .length       (length),
      .parity_enable(parity_enable),
      .parity_even  (parity_even),
      .parity_stick (parity_stick)
  );
  always @(posedge clk) begin
    past_valid <= 1;
    if (!nrst) begin
      phase        <= 0;
      delivered    <= 0;
      start_offset <= 0;
    end else begin
      phase <= phase + 1'b1;
      if (phase == 0) begin
        delivered    <= 0;
        start_offset <= 0;
      end
      if (receiver.active && receiver.age == 0 && !delivered) start_offset <= phase;
      if (receiver.strobe) begin
        assert (!delivered);
        delivered <= 1;
        assert (receiver.data_out == (frame_byte & data_mask));
        assert (receiver.parity_out == (parity_enable && wire_parity));
        assert (receiver.errors == {!(|(frame_byte & data_mask)) &&
                  !(parity_enable && wire_parity) && bad_stop,
                  bad_stop, parity_enable && bad_parity});
      end
    end
    if (past_valid && nrst) begin
      if ($past(nrst)) assume (frame_config == $past(frame_config));
      if (phase < 32) assert (!receiver.active && !receiver.strobe);
      if (phase != 0 && phase < 38) assert (!delivered);
      // Boundary noise after delivery can start only a rejected start glitch;
      // the following mark interval must return the receiver to idle.
      if (delivered && receiver.active) assert (receiver.age < 8);
      if ({1'b0, phase} >= frame_end + 16) assert (!receiver.active && !receiver.strobe);
      if (phase >= 38) begin
        assert (start_offset == 35 || start_offset == 36);
        assert (delivered == ({1'b0, phase} > delivery_phase));
        if ({1'b0, phase} < delivery_phase) assert (receiver.active);
        if ({1'b0, phase} < frame_end)
          assert (receiver.strobe == ({1'b0, phase} == delivery_phase));
      end
      if (receiver.active && !delivered && !receiver.strobe && phase >= 38) begin
        assert ({1'b0, phase} == {1'b0, receiver.age} + {1'b0, start_offset});
        if (!parity_enable || int'(receiver.age) < 8 + 16 * (data_bits + 1)) begin
          assert (!receiver.parity_slot && !receiver.parity_error);
        end else begin
          assert (receiver.parity_slot == wire_parity);
          assert (receiver.parity_error == bad_parity);
        end
        for (int i = 0; i < 8; i++) begin
          if (i < data_bits && int'(receiver.age) >= 24 + 16 * i)
            assert (receiver.payload[i] == frame_byte[i]);
            else assert (!receiver.payload[i]);
        end
      end
      cover (receiver.strobe && frame_byte == 8'hA5 && receiver.errors == 0);
      cover (receiver.strobe && length == 0 && parity_enable && bad_parity);
      cover (receiver.strobe && receiver.errors[2]);
    end
  end
endmodule
