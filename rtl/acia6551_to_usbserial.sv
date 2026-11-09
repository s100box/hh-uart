// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// acia6551_to_usbserial -- 6551 ACIA <-> usb_serial_if adapter.
//
//   Lets a 6551 ACIA core drive usb_serial_chan instead of a TX/RX serializer
//   pair, so host software talks to a USB serial device through the ACIA's
//   unchanged control, status and data registers. The ACIA side is the
//   transmit-FIFO head (tx_byte/tx_valid, popped by tx_consumed) and the
//   receive-FIFO write port (rx_byte/rx_strobe, refused while rx_full).
//
//   TX: the ACIA holds tx_valid while its TX FIFO is non-empty, with tx_byte
//   at its head; the channel takes a byte whenever its TX FIFO has room
//   (us.tx_ready), and tx_consumed pops the ACIA on that same cycle, so one
//   byte moves per cycle and a full channel FIFO backpressures the ACIA. RX:
//   a byte from the USB device is written into the ACIA's RX FIFO whenever it
//   has room (rx_full_i low); the ACIA's own RDRF and overrun status are the
//   host's flow control. Purely combinational.
//============================================================================
`default_nettype none

module acia6551_to_usbserial (
    // ---- 6551 ACIA side ----
    input  wire logic [7:0] tx_byte_i,      // <- acia_6551.tx_byte
    input  wire logic       tx_valid_i,     // <- acia_6551.tx_valid (level)
    output logic            tx_consumed_o,  // -> acia_6551.tx_ready (1-cycle pulse)
    output logic      [7:0] rx_byte_o,      // -> acia_6551.rx_byte
    output logic            rx_strobe_o,    // -> acia_6551.rx_strobe (1-cycle)
    input  wire logic       rx_full_i,      // <- acia_6551.rx_full

    // ---- USB bridge backend ----
    usb_serial_if.core us
);

  // ---- TX: 6551 head -> channel, one byte per cycle while room exists ------
  assign us.tx_data    = tx_byte_i;
  assign us.tx_valid   = tx_valid_i;
  assign tx_consumed_o = tx_valid_i & us.tx_ready;  // dequeue when the FIFO took it

  // ---- RX: inject into the 6551 whenever it has room ----------------------
  wire inject = us.rx_valid & ~rx_full_i;
  assign rx_byte_o   = us.rx_data;
  assign rx_strobe_o = inject;
  assign us.rx_ready = inject;
  // (us.rx_err is driven by the channel; the 6551 has no rx_err input to take it)

  // ---- Baud sideband not forwarded (control reg not exposed as a port) -----
  assign us.mctl_out = 2'b11;  // 6551: no modem lines; hold DTR/RTS asserted
  assign us.baud_div = 32'd0;
  assign us.baud_stb = 1'b0;

endmodule

`default_nettype wire
