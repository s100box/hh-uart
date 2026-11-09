// SPDX-License-Identifier: MIT OR Apache-2.0
// SPDX-FileCopyrightText: 2026 Howard M. Harte
//============================================================================
// usb_serial_if -- the canonical host-UART <-> USB-bridge byte seam.
//
//   One AXI-Stream-style byte channel each way (level valid/ready both ways, a
//   superset of the pulse-ack style the host UART cores already use), plus an
//   optional baud sideband. A retro host UART core (a Z180 ASCI, a 6551 ACIA)
//   drives the `core` modport through a small adapter of its own;
//   `usb_serial_chan` consumes the `chan` modport and presents a byte register
//   window to the processor running the USB host stack.
//
//   TX = core -> backend (host bytes destined for the USB serial device).
//   RX = backend -> core (bytes the USB serial device produced).
//   A transfer happens on any cycle both valid and ready are high.
//============================================================================
`default_nettype none

interface usb_serial_if;
  logic [ 7:0] tx_data;  // core -> backend byte
  logic        tx_valid;  // core has a byte
  logic        tx_ready;  // backend can accept (TX FIFO not full)

  logic [ 7:0] rx_data;  // backend -> core byte
  logic        rx_valid;  // backend has a byte (RX FIFO not empty)
  logic        rx_ready;  // core can accept

  logic [ 1:0] rx_err;  // {PE, FE} for rx_data; USB-sourced RX ties this 0

  logic [31:0] baud_div;  // host divisor word (e.g. CNTLB-derived); optional
  logic        baud_stb;  // one host-domain pulse: baud_div changed

  // Modem-control sideband (levels, host domain; both optional -- tie
  // mctl_out 0 and ignore mctl_in when the host UART has no modem lines).
  logic [ 1:0] mctl_out;  // host -> USB adapter: {RTS, DTR} (asserted-high)
  logic [ 3:0] mctl_in;  // USB adapter -> host: {DCD, RI, DSR, CTS}

  // Host UART core side (the adapter drives this).
  modport core(
      output tx_data, tx_valid,
      input tx_ready,
      input rx_data, rx_valid,
      output rx_ready,
      input rx_err,
      output baud_div, baud_stb,
      output mctl_out,
      input mctl_in
  );

  // Backend side (usb_serial_chan drives this).
  modport chan(
      input tx_data, tx_valid,
      output tx_ready,
      output rx_data, rx_valid,
      input rx_ready,
      output rx_err,
      input baud_div, baud_stb,
      input mctl_out,
      output mctl_in
  );
endinterface

`default_nettype wire
