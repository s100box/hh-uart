# hh-uart design

<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Howard M. Harte -->

For setup, build commands, and commit checks, see the [README](../README.md).
This document defines the design and its verification scope.

**Contents**

- [1. Purpose](#1-purpose)
- [2. The 16x timebase](#2-the-16x-timebase)
- [3. `uart_rx_core` — receiver](#3-uart_rx_core--receiver)
- [4. `uart_tx_core` — transmitter](#4-uart_tx_core--transmitter)
- [5. `uart_baud_dds` — baud generator](#5-uart_baud_dds--baud-generator)
- [6. The USB-serial seam](#6-the-usb-serial-seam)
- [7. Building a UART from the cores](#7-building-a-uart-from-the-cores)
- [8. Resource and timing character](#8-resource-and-timing-character)
- [9. Verification](#9-verification)

## 1. Purpose

Vendor-neutral SystemVerilog building blocks for a system-on-chip's serial
console: the receiver, transmitter and baud generator a register-compatible
UART (16550, Z180 ASCI, 6551 ACIA, Z80 DART) is built from, and the byte-channel
seam that carries that UART's traffic to a processor running a USB host stack
instead of to a pin. `uart16550` combines the serial cores with an
8250/16450/16550/16550A register file, FIFOs, interrupts and modem control.
Other register-compatible UARTs can use the RX, TX and baud cores directly,
tying their frame-configuration inputs to a line-control register or constants.

| Module | File | Role |
|---|---|---|
| `uart_rx_core` | `rtl/uart_rx_core.sv` | 16x-oversampling receiver, runtime frame config, error flags |
| `uart_tx_core` | `rtl/uart_tx_core.sv` | transmitter with an optional one-byte holding buffer, runtime frame config, break |
| `uart_baud_dds` | `rtl/uart_baud_dds.sv` | exact-average baud generator: DDS to a reference rate, then /divisor and /16 |
| `uart16550` | `rtl/uart16550.sv` | complete 8250-family UART; `VARIANT` selects 8250, 16450, 16550 or 16550A; the module header documents the bus and variant behavior |
| `usb_serial_if` | `rtl/usb_serial_if.sv` | interface: valid/ready byte channel each way plus baud and modem-control sidebands |
| `usb_serial_fifo` | `rtl/usb_serial_fifo.sv` | byte FIFO, single-clock or Gray-pointer asynchronous by parameter |
| `usb_serial_chan` | `rtl/usb_serial_chan.sv` | the USB-bridge backend: a FIFO pair and an eight-register window for the stack |
| `acia6551_to_usbserial` | `rtl/acia6551_to_usbserial.sv` | combinational adapter from a 6551 ACIA's FIFO ports onto the `core` modport |

Module, port and parameter names are public API; downstream designs
instantiate them by name. Every clocked module uses a synchronous, active-low
reset (`nrst_i`); `acia6551_to_usbserial` is combinational. No module has a
vendor primitive.

## 2. The 16x timebase

The three UART cores share one timing contract, and it is the reason the
receiver works at a fractional number of clocks per bit.

`uart_baud_dds` produces two ticks. `samp16_tick_o` is a one-clock pulse
sixteen times per bit period; `bit_tick_o` is a one-clock pulse once per bit
period, derived from `samp16_tick_o` by a structural /16 counter. There are
therefore exactly sixteen sample ticks between consecutive bit ticks during
steady operation, whatever the core/reference ratio. Reset or `resync_i`
restarts the /16 count; it does not restart the free-running sample divider.

The receiver consumes only `samp16_tick_o`. It recovers each bit by counting
sixteen sample ticks from the previous sample point and sampling at the
eighth, so it never measures the bit period and never depends on `clk_i`
being an integer multiple of the baud rate. A receiver that instead measures
the bit period from a 1x tick into an integer register, re-latched every bit,
sees that measurement jitter between adjacent integers at a fractional ratio
(40.5 MHz / 115200 = 351.5625 clocks per bit), can move its "equal to target"
sample point below the running count mid-frame, and then stalls for a full
counter wrap. Counting on the 16x grid has no such failure mode.

The transmitter consumes only `bit_tick_o`: its FSM advances one bit per
tick, so each bit on the wire is one tick period long. It uses four states:
idle, data, parity and stop. Starting a frame emits the start bit while entering
the data state; there is no separate start state.

A UART therefore wires one `uart_baud_dds` to one receiver and one
transmitter. Both ride the same generator, so the /16 stage's fixed
one-clock phase offset between the two ticks is immaterial.

## 3. `uart_rx_core` — receiver

### 3.1 Ports

| Port | Direction | Meaning |
|---|---|---|
| `clk_i`, `nrst_i` | in | clock, synchronous active-low reset |
| `samp16_tick_i` | in | the 16x sample tick (section 2) |
| `rxd_i` | in | the serial line, idle high; asynchronous to `clk_i` |
| `word_len_i[1:0]` | in | data bits: 0 = 5, 1 = 6, 2 = 7, 3 = 8 |
| `parity_en_i` | in | a parity slot is present in the frame |
| `parity_even_i` | in | 1 even, 0 odd (when not stick) |
| `parity_stick_i` | in | the slot holds a fixed bit: `~parity_even_i` (1 = mark, 0 = space) |
| `data_o[7:0]` | out | the byte, LSB-first on the wire, right-aligned; unused high bits are 0 |
| `err_o[2:0]` | out | `{BI, FE, PE}` for `data_o` |
| `parity_bit_o` | out | the raw bit sampled in the parity slot; 0 when the frame has no slot |
| `stb_o` | out | one-clock pulse: `data_o`, `err_o`, `parity_bit_o` are fresh |

The config inputs are sampled live, bit by bit, not latched per frame; the
consuming UART holds them stable while a frame is in flight.

### 3.2 Line input

A two-flop synchronizer takes `rxd_i` into the clock domain, and a third flop
delays it one more clock so that a start bit is detected as a **falling edge**
(`rxd_sync_d1 & ~rxd_sync`), not as a low level. A line that is merely still
low, the tail of a framing-error stop bit or a break, cannot be taken for a
fresh start bit; the next frame begins only when the line has returned to
mark and fallen again.

### 3.3 Frame recovery

The FSM has five states, in a three-bit register:

| State | Entered on | Leaves after |
|---|---|---|
| `StIdle` | reset, end of every frame | a falling edge on the line |
| `StStart` | the falling edge | 8 sample ticks: the line is re-read at mid-start; low confirms the start, high is a glitch and returns to idle with no strobe |
| `StData` | a confirmed start | 16 ticks per bit, sampling mid-bit, LSB first into `rx_sr_q[bit_idx]`; after bit `ndata-1` goes to `StParity` if `parity_en_i`, else `StStop` |
| `StParity` | the last data bit | 16 ticks: the slot bit is sampled and compared with the expected parity |
| `StStop` | the parity or last data bit | 16 ticks: the stop bit is sampled, the byte and flags are presented, `stb_o` pulses, back to idle |

The sample-count constants are `SampMid = 7` (the eighth tick after the edge
is the middle of the start bit) and `SampFull = 15` (sixteen ticks from one
sample point to the next). `StIdle` clears the per-frame state: the shift
register, the parity error, the parity-slot bit and the break tracker.

Every assignment to the state register is one of the five named codes;
codes 5 to 7 are unreachable from the ports (proved, section 9.3), and the
case statement's `default` arm returns them to idle so a corrupted register
recovers.

### 3.4 Parity and the flags

The expected parity is computed over the recovered data masked to `ndata`
bits (`data_mask = 8'hFF >> (8 - ndata)`):

- even: the XOR of the data bits; odd: its complement;
- stick: `~parity_even_i`, so `parity_stick_i & ~parity_even_i` is mark
  (always 1) and `parity_stick_i & parity_even_i` is space (always 0), the
  16550 LCR encoding.

The flags presented with `stb_o`, from the sample at mid-stop:

| Flag | Set when |
|---|---|
| `PE` (`err_o[0]`) | the frame has a parity slot and the sampled bit differs from the expected one |
| `FE` (`err_o[1]`) | the stop bit sampled low |
| `BI` (`err_o[2]`) | every data bit, the parity slot if present, and the stop bit sampled low |

`BI` is tracked by `zero_q`, set at frame start and cleared by any 1 sampled
in a data or parity position, so a zero byte with a 1 in its parity slot and a
low stop bit is a framing error, not a break. The errored character is still
delivered; a consumer that wants only `{PE, FE}` takes `err_o[1:0]`.

A long break (the line low for many frame times) delivers exactly one break
character: after the strobe the receiver is idle, and the still-low line is
not a falling edge.

### 3.5 The parity-slot bit

`parity_bit_o` is the raw bit from the slot, held with `data_o`. For ordinary
parity it says nothing `PE` does not. It exists for protocols that put data
in that position: the Z180 ASCI's multiprocessor mode frames a byte as start,
7 or 8 data bits, the MPB bit, and the stop bits, with no provision for
parity. Such a consumer drives `parity_en_i` from its own mode bit, reads the
ninth bit here, and ignores `PE`. The output is 0 for a frame with no slot
(cleared per frame, not left at the previous frame's value).

### 3.6 Timing

The strobe lands at mid-stop, half a bit period before the stop bit ends, so
the consumer has that half period plus the next start bit before the next
byte can begin. The synchronizer and edge detector add three clocks between
the line's edge and the FSM seeing it; at any usable ratio that is a small
fraction of the eight ticks to mid-start and does not move the sample point
out of the bit.

## 4. `uart_tx_core` — transmitter

### 4.1 Parameter and ports

| Parameter | Default | Meaning |
|---|---|---|
| `HOLD` | 1 | 1 = a one-byte holding buffer in front of the shifter; 0 = none, for a parent that keeps its waiting bytes itself (section 4.2) |

| Port | Direction | Meaning |
|---|---|---|
| `clk_i`, `nrst_i` | in | clock, synchronous active-low reset |
| `baud_tick_i` | in | the per-bit tick (section 2) |
| `data_i[7:0]`, `we_i` | in | the byte and a one-clock enqueue strobe |
| `word_len_i`, `parity_en_i`, `parity_even_i`, `parity_stick_i` | in | the frame config, encoded as the receiver's |
| `stop2_i` | in | 1 = two stop bits, 0 = one |
| `break_i` | in | force the line to spacing (low) |
| `ready_o` | out | `we_i` will be taken this clock: the holding buffer is free (`HOLD` = 1), or this is the clock the shifter loads (`HOLD` = 0) |
| `tx_idle_o` | out | the shifter is idle **and** the holding buffer, if any, is empty |
| `txd_o` | out | the serial line, idle high |

### 4.2 Holding buffer and handshake

With `HOLD` = 1, a one-byte holding buffer (`hold`, `hold_valid`) decouples
the producer from the line. `we_i` with `ready_o` high loads the buffer in
one clock and drops `ready_o`; `we_i` with `ready_o` low is refused and the
byte is lost, so a producer writes only when `ready_o` is high. On the baud
tick that begins a frame the buffer empties into the shifter and `ready_o`
returns high while the frame is still on the wire, so a producer that
refills on every `ready_o` keeps the line continuously busy. A write on the
same clock as that transfer is refused like any other write while the
buffer is full (`ready_o` is low on that clock).

`tx_idle_o` is the signal for "everything has left": it is high only when the
FSM is in `StIdle` and the buffer is empty. A UART whose holding register is
this buffer takes its transmitter-empty status bit from `tx_idle_o` and its
holding-register-empty bit from `ready_o`.

With `HOLD` = 0 there is no buffer, for a UART with its own holding register
or FIFO that must keep a waiting byte until its start bit (to replace it, or
discard it on a clear: section 7.1). `ready_o` is high only in the load clock,
a baud tick with the FSM in `StIdle` or the frame's last stop bit done; a
byte offered with `we_i` in that clock enters the shifter and its start bit
begins on that tick, and a `we_i` in any other clock is ignored. The parent
offers its waiting byte combinationally while `ready_o` is high and gives it
up in that clock. Frames still run back to back, the config is sampled at
the load as with the buffer, and `tx_idle_o` is the FSM in `StIdle`.

### 4.3 Frame emission

The FSM advances only on `baud_tick_i`, so `txd_o` is one bit per tick:

| State | Emits | Then |
|---|---|---|
| `StIdle` | mark; if a byte is waiting (in the buffer, or offered now with `HOLD` = 0), the **start bit** (low) | latches the byte, `ndata`, the parity bit and `parity_en_i`; `StData` |
| `StData` | `sr_q[bit_idx]`, LSB first | after bit `ndata-1`: loads the stop count from `stop2_i`; `StParity` if the frame has parity, else `StStop` |
| `StParity` | the latched parity bit | `StStop` |
| `StStop` | mark for one or two ticks (the stop count) | then, if a byte is waiting, the next start bit at once (back to `StData`); else mark and `StIdle` |

Two consequences of this shape:

- **The data-bit count and the parity bit are latched when the byte loads**
  (`ndata_q`, `par_q`, `paren_q`), so a change of `word_len_i` or the parity
  inputs mid-frame does not corrupt the byte in flight. `stop2_i` is sampled
  at the last data bit, not at load.
- **Stop bits occupy full bit periods.** A queued byte's start bit begins on
  the tick after the last stop tick, never squashing a stop bit, so the
  second frame starts exactly `(1 + ndata + parity + stops)` bit periods after
  the first.

The parity bit for the waiting byte is computed combinationally from it
(`hold`, or `data_i` with `HOLD` = 0) and the live config (`par_load`) and
latched at load, with the same even/odd/stick rules as the receiver's
expected parity.

### 4.4 Break

`txd_o = break_i ? 0 : txd_q`: break is a combinational override of the
line, not a state. The FSM keeps running underneath, so a frame in flight
finishes (its bits unseen on the wire) and `tx_idle_o` behaves as usual;
releasing `break_i` returns the line to whatever the FSM is emitting. A UART
that must send a clean break after draining its queue waits for `tx_idle_o`
before asserting `break_i`.

### 4.5 Reset

Reset returns the line to mark, empties the buffer and the shifter
(`tx_idle_o` high, and `ready_o` high with the buffer) on the next clock,
whatever was in flight.

## 5. `uart_baud_dds` — baud generator

### 5.1 Parameters and ports

| Name | Meaning |
|---|---|
| `CLK_HZ` (parameter, `int unsigned`, default 48 000 000) | the core clock rate |
| `REF_HZ` (parameter, `int unsigned`, default 1 843 200) | the UART reference rate the DDS synthesizes; must not exceed `CLK_HZ` |
| `divisor_i[15:0]` | baud = `REF_HZ / (16 * divisor_i)`; 0 disables both ticks |
| `resync_i` | pulse: restart the /16 bit phase |
| `samp16_tick_o` | the 16x sample tick, free-running |
| `bit_tick_o` | the per-bit tick |

`CLK_HZ` must match the actual input clock frequency; it is an elaboration-time
parameter, not a measurement of a dynamically changing clock. Different builds
can change that clock and parameter together while retaining the same firmware
divisor mapping.

With the defaults, divisor 1 is 115200 baud, 3 is 38400, 12 is 9600: the
16550's divisor latch, so a 16550-compatible UART drives `divisor_i` from
`{DLM, DLL}` directly and a fixed-rate UART ties it to a constant.

### 5.2 Stage 1: the DDS

A 32-bit phase accumulator adds `REF_HZ` every clock; when the sum reaches
`CLK_HZ` it subtracts `CLK_HZ` and pulses `ref_tick`. Over any whole number of
seconds the tick count is exactly `REF_HZ` per second, whatever `CLK_HZ` is,
with at most one clock of jitter on any individual tick. An integer
`CLK_HZ / baud` divider cannot do this (48 MHz has no integer divisor for
115200); the accumulator's error is bounded and does not drift.

The accumulator never holds a value at or above `CLK_HZ` (an inductive
invariant: a sum below `CLK_HZ` is kept, a sum at or above it loses `CLK_HZ`
and is then below `REF_HZ`, which is at most `CLK_HZ`). It is 32 bits wide so
the parameter can be any `int unsigned`; bits above log2(`CLK_HZ`) are
constant 0.

### 5.3 Stage 2: the divider and the /16

`samp_cnt_q` counts `ref_tick`s; when `samp_cnt_q + 1 >= divisor_i` it
resets and pulses `samp16_tick_o`, so the sample tick runs at
`REF_HZ / divisor_i` (16 times the baud rate). The compare is `>=`, not
`==`, so a divisor lowered below the running count fires on the next
reference tick instead of running the counter around; a divisor of 0 holds
the sample counter at 0 and suppresses both registered tick outputs after the
next clock edge. The /16 counter preserves its partial phase while disabled,
unless `resync_i` restarts it. A sample pulse left in the pipeline when the
divisor becomes zero is not consumed by the /16 stage.

`bit_cnt_q` counts sample ticks 0 to 15 and pulses `bit_tick_o` on the
sixteenth. `resync_i` resets that count with priority over a sample tick on
the same clock. The next `bit_tick_o` follows sixteen subsequently consumed
sample pulses. The sample divider remains free-running, so this does not
guarantee a full nominal bit period measured from the resync edge; the first
sample pulse can arrive immediately afterward. A consumer starting TX outside
`bit_tick_o` must account for that phase. `uart_tx_core` starts and advances
only on `bit_tick_o`, so its generator ties `resync_i` to 0.
`samp16_tick_o` is unaffected, preserving the receiver's sample grid.

Each output is registered one clock after the event that produces it.

## 6. The USB-serial seam

A retro host UART whose "line" is a USB serial adapter needs its bytes
carried to the processor that runs the USB host stack. Three modules make
that path.

```text
host UART core --adapter--> usb_serial_if.core | .chan --> usb_serial_chan --> stack register window
                                                               |  TX FIFO: host -> stack
                                                               |  RX FIFO: stack -> host
                                                               |  baud + modem-control sidebands
```

TX is host to backend (bytes for the USB device); RX is backend to host
(bytes the device produced).

### 6.1 `usb_serial_if`

An interface carrying one AXI-Stream-style byte channel each way (level
`valid`/`ready`; a byte moves on any clock both are high) plus sidebands,
all in the host core's clock domain:

| Signal | Direction | Meaning |
|---|---|---|
| `tx_data[7:0]`, `tx_valid`, `tx_ready` | core → chan, ready back | a byte for the USB device |
| `rx_data[7:0]`, `rx_valid`, `rx_ready` | chan → core, ready back | a byte from the USB device |
| `rx_err[1:0]` | chan → core | `{PE, FE}` for `rx_data`; `usb_serial_chan` ties it 0 |
| `baud_div[31:0]`, `baud_stb` | core → chan | the host's divisor word and a one-clock "it changed" pulse |
| `mctl_out[1:0]` | core → chan | `{RTS, DTR}`, asserted high |
| `mctl_in[3:0]` | chan → core | `{DCD, RI, DSR, CTS}`, asserted high |

The `core` modport is what the host UART's adapter drives; the `chan`
modport is what `usb_serial_chan` drives. A host with no modem lines ties
`mctl_out` to 0 and ignores `mctl_in`; one with a fixed rate leaves
`baud_stb` at 0.

### 6.2 `usb_serial_fifo`

One module, two builds chosen by `SAME_CLOCK`. `DEPTH` is a power of two,
at least 4 with `SAME_CLOCK = 0` or 2 with `SAME_CLOCK = 1`. Smaller values
leave invalid pointer slices in the selected implementation. `DW` is positive;
the pointers are `$clog2(DEPTH) + 1` bits wide so full and empty are
distinguished by the extra bit. `rdata_o` is the asynchronous read of the
head entry (`mem[rd]`), which a consumer may register; `re_i` advances the
read pointer. A write while full and a read while empty are ignored.

- **`SAME_CLOCK = 1`**: binary pointers, combinational full and empty from
  the pointer compare, one `always_ff` on the write clock; `rd_clk_i` and
  `rd_nrst_i` are unused and tied to the write side's. Lowest latency and no
  clock-domain crossing.
- **`SAME_CLOCK = 0`**: a Cummings-style asynchronous FIFO. Each side keeps a
  binary pointer and its Gray code; each Gray pointer crosses to the other
  domain through a two-flop synchronizer; full (write side) and empty (read
  side) are **registered** from the next-pointer compares, so the increment
  enables depend only on a registered flag and there is no combinational
  loop. Full is "next write Gray equals the synchronized read Gray with its
  top two bits inverted"; empty is "next read Gray equals the synchronized
  write Gray". Empty resets to 1.

In the asynchronous build the flags are conservative: a byte written on one
side becomes visible on the other two to three destination clocks later,
after the synchronizers, so the FIFO may retain full or empty after the other domain has freed space or
written data. That conservative delay can stall traffic but does not admit a
write into a locally full FIFO or a read from a locally empty FIFO.

**Constraint contract.** The consuming design must bound physical delay and
inter-bit skew from each Gray source register to the first synchronizer stage
(`rd_gray_q` to `rd_gray_m`, `wr_gray_q` to `wr_gray_m`) so successive pointer
changes cannot overlap at the receiver. A bound no greater than the shorter
clock period is a conservative choice; derive the actual constraints from
the two clocks and the target tool's CDC guidance. A false path alone supplies
no skew bound. Keep each first-to-second synchronizer stage normally timed,
and check that clock-group or false-path exceptions do not override a required
maximum-delay constraint. AMD's [Gray-bus example](https://docs.amd.com/r/en-US/ug903-vivado-using-constraints/set_bus_skew-Example-Two)
shows the separate roles of bus skew and datapath-only maximum delay.
Both domains' resets must be asserted together at power-up; a reset of one
side alone leaves the other side's synchronized copy of its pointer stale.

### 6.3 `usb_serial_chan`

Parameters `DEPTH` (both FIFOs) and `SAME_CLOCK` (passed to both FIFOs and
selecting the FIFO implementations). Sideband synchronizer pipelines remain
present in both builds, including their destination-clock latency. The host side is `host_clk_i`
and `host_nrst_i` with the `chan` modport; the stack side is `v_clk_i`,
`v_nrst_i` and a byte-wide register window with one-clock `v_we_i` and
`v_re_i` strobes and a combinational `v_rdata_o`.

**The FIFO pair.** The TX FIFO is written by the host (`tx_valid & ~full`,
with `tx_ready = ~full`) and read by the stack's DATA_TX reads. The RX FIFO
is written by the stack's DATA_RX writes and read by the host
(`rx_valid & rx_ready`, with `rx_valid = ~empty`). Each FIFO's write clock
is its producer's domain, so under `SAME_CLOCK = 0` both are true
asynchronous FIFOs crossing in opposite directions.

**The register window** (`v_addr_i[2:0]`):

| Addr | Name | Access | Contents |
|---|---|---|---|
| 0 | STATUS | R | bit 0 TX_AVAIL (a host byte waits), bit 1 RX_SPACE (the RX FIFO is not full), bit 2 BAUD_PENDING |
| 1 | DATA_TX | R | the oldest host byte; the read pops it |
| 2 | DATA_RX | W | push a byte toward the host |
| 3–6 | BAUD0–BAUD3 | R | the latched 32-bit divisor word, LSB first; reading BAUD3 clears BAUD_PENDING |
| 7 | MCTL | R | bit 7 MCTL_PENDING, bits 1:0 `{RTS, DTR}`; the read clears the pending flag |
| 7 | MCTL | W | bits 3:0 `{DCD, RI, DSR, CTS}` toward the host; bits 7:4 ignored |

Addresses not in the table read 0. A DATA_TX read with TX_AVAIL clear is
harmless (the FIFO ignores the pop) and returns whatever the empty FIFO's
head cell holds; a DATA_RX write with RX_SPACE clear is dropped.

**The baud sideband.** On `baud_stb` the host domain latches `baud_div` and
flips a toggle. The stack domain synchronizes the toggle through two flops
and detects its edge against a third; on that edge it copies the latched
word (stable for at least two stack clocks by then) and sets BAUD_PENDING,
which a BAUD3 read clears. A second `baud_stb` before the stack has read the
word can replace it. A new capture wins over a same-cycle acknowledgement.
There is no source acknowledgement or event queue: keep the source word stable
until capture, with the stack clock running. Faster toggles may coalesce or
cancel, and four byte reads are not an atomic snapshot during another capture.
The stack uses the captured word to program the adapter's line rate.

**The modem-control sideband.** Outbound, the host's `{RTS, DTR}` levels are
synchronized into the stack domain through two flops, a change against a
third flop sets MCTL_PENDING, and an MCTL read returns the levels and clears
the flag. Inbound, an MCTL write latches `{DCD, RI, DSR, CTS}` in the stack
domain and two flops carry them back to the host as `mctl_in`. The inbound
latch and its synchronizer **reset asserted** (`4'b1111`), so a host sees
ready modem lines until the stack knows better. The bits represent independent modem indications rather than an atomic word;
a source update can appear over several destination clocks.

### 6.4 `acia6551_to_usbserial`

The adapter for a 6551 ACIA core, combinational. On the ACIA side it takes
the transmit FIFO's head (`tx_byte_i`, with `tx_valid_i` high while the FIFO
holds a byte) and the receive FIFO's state (`rx_full_i`), and drives
`tx_consumed_o` (pop the transmit FIFO) and `rx_byte_o`/`rx_strobe_o` (write
the receive FIFO).

| Direction | Rule |
|---|---|
| TX | `tx_valid` = `tx_valid_i`; `tx_consumed_o` = `tx_valid_i & tx_ready`, so the ACIA pops in the cycle the channel takes the byte, one byte per cycle, and a full channel FIFO holds the rest in the ACIA |
| RX | `rx_strobe_o` = `rx_ready` = `rx_valid & ~rx_full_i`; a byte waits in the channel while the ACIA's FIFO is full, and the ACIA's own RDRF and overrun status are the host's flow control |
| Sidebands | `mctl_out` = `2'b11` (the 6551 has no modem-control outputs, so DTR and RTS read asserted); `baud_stb` = 0 and `baud_div` = 0 (the control register is not a port); `rx_err` and `mctl_in` are unused |

## 7. Building a UART from the cores

A fixed-rate 8N1 console: one `uart_baud_dds` with `divisor_i` a constant
and `resync_i` 0; `uart_rx_core` on `samp16_tick_o` and `uart_tx_core` on
`bit_tick_o`, both with `word_len_i = 3`, `parity_en_i = 0`, `stop2_i = 0`,
`break_i = 0`. The UART's receive-data-ready flag is set by `stb_o` and
cleared by its data read; its transmit-ready flag is `ready_o`; its
transmitter-empty flag is `tx_idle_o`.

A 16550-class UART adds its line-control register on the config inputs of
both cores (word length, parity enable/even/stick, stop bits, break), its
divisor latch on `divisor_i`, and takes the receiver's `err_o` into its line
status register (`BI`, `FE`, `PE`) alongside the overrun flag it computes
itself from an unread `stb_o`. A Z180 ASCI in multiprocessor mode drives
`parity_en_i` from its MPE bit and reads `parity_bit_o` as the MPB bit.

A USB-serial bridge replaces the pin with the seam of section 6: the UART's
adapter presents its transmit bytes on `tx_data`/`tx_valid` when the host
writes its transmit register, takes `rx_data` with `rx_ready` into its
receive register, pulses `baud_stb` when the host writes its divisor, and
maps its modem-control outputs and inputs onto `mctl_out` and `mctl_in`.
`usb_serial_chan` sits behind the interface with `SAME_CLOCK` chosen by
whether the host core and the stack share a clock.

### 7.1 `uart16550` — the 8250-family UART

`uart16550` is that 16550-class UART, complete: the National/TI 8250-family
register file over one `uart_baud_dds` (`resync_i` 0), one `uart_tx_core`
and one `uart_rx_core`, with LCR on both cores' frame inputs and `{DLM, DLL}`
on `divisor_i`.

| Parameter | Default | Meaning |
|---|---|---|
| `VARIANT` | 3 | 0 = 8250 (no scratch register, no FIFO); 1 = 16450 (scratch); 2 = 16550 (FCR0 sets IIR D7:6 = `10` but nothing is buffered); 3 = 16550A (16-byte FIFOs, IIR D7:6 = `11`). On 0-2 the FIFO arrays constant-propagate away |
| `CLK_HZ`, `REF_HZ` | 70 588 235, 1 843 200 | core clock and reference crystal; baud = `REF_HZ` / (16 × divisor) whatever `CLK_HZ` is |
| `IRQ_VECTOR` | `8'hFF` | the constant on `int_vec_o`, for a Z80/Z180 IM2 acknowledge |
| `RESET_DIVISOR` | `16'h0000` | reset value of `{DLM, DLL}`; 0 leaves the baud ticks off until firmware programs a divisor, a nonzero value makes `baud_*_tick_o` live out of reset |

The CPU side is decoded by the SoC: `cs_i`, `addr_i[2:0]` and one-clock
`rd_i` / `wr_i` strobes; `dout_o` is combinational and valid while the strobe
is high, and a read's side effects (an RBR pop, the LSR/MSR clears, the THRE
clear) apply at the clock edge that ends it. The modem inputs `cts_n_i`,
`dsr_n_i`, `dcd_n_i`, `ri_n_i` are active low and two-flop synchronized;
`rts_n_o` and `dtr_n_o` are active low. `baud_bit_tick_o` and
`baud_samp16_tick_o` are the baud generator's ticks, for a second UART that
slaves its line rate to this one.

| Offset | DLAB | Read | Write |
|---|---|---|---|
| 0 | 0 | RBR (RX FIFO top) | THR (TX FIFO) |
| 0 | 1 | DLL | DLL |
| 1 | 0 | IER (D3:0) | IER |
| 1 | 1 | DLM | DLM |
| 2 | — | IIR | FCR |
| 3 | — | LCR | LCR |
| 4 | — | MCR (D4:0) | MCR |
| 5 | — | LSR | ignored |
| 6 | — | MSR | ignored |
| 7 | — | SCR (0 on the 8250) | SCR |

- **Interrupts.** IIR reports the highest enabled source: line status
  (`06`), then received data (`04`, DR in 16450 mode, the RX FIFO at or above
  its trigger level in FIFO mode) or the character timeout (`0C`), then THRE
  (`02`), then modem status (`00`); `01` is none, and D7:6 carry the FIFO
  status. Each source has its IER enable. `int_o` is a pending source gated
  by OUT2, and loopback opens the gate.
- **THRE.** The THRE interrupt is set at reset, when THRE rises, by an IER
  write with ETBEI set while THRE is set, and by a TX FIFO clear (an FCR0
  change among them), which sets THRE. A THR write clears it, and so does
  an IIR read that reports it; a read reporting a higher source does not. In
  16450 mode THRE's rise sets it at once. In FIFO mode it does so only if two
  bytes have been in the TX FIFO at once, or FCR0 has changed, since THRE
  last rose; otherwise the interrupt is held back a character time less the
  last stop bit (the frame's bits from start to stop, 1.5 stop bits counted
  as 2, less one), so it comes as the last byte's final stop bit begins. A
  THR write during the hold-back cancels it, and an ETBEI write or a TX FIFO
  clear during it sets the interrupt at once and ends it. The LSR THRE bit is
  never held back.
- **Line status.** OE, PE, FE and BI are sticky until an LSR read, and a flag
  raised in the clock of that read survives it. In 16450 mode the flags are
  set as a byte lands in RBR; a byte landing on an unread RBR replaces it and
  sets OE, but one landing in the clock the RBR is read replaces it without
  OE. In FIFO mode a character's PE, FE and BI reach the LSR the clock after
  it reaches the top of the RX FIFO, so a driver that reads LSR and then RBR
  attributes them to that byte. FIFOE (D7) is set while at least one
  character in the RX FIFO carries a PE, FE or BI, and an LSR read clears it
  once none remains; an errored character landing in the clock of that read
  keeps it set. An RX FIFO clear leaves FIFOE to the next LSR read, and
  leaving FIFO mode clears it. A byte completing with the RX FIFO full is
  dropped with OE, unless an RBR read frees a place in the same clock, when
  the byte takes it. THRE is set while the THR (TX FIFO) is empty; TEMT is
  THRE with the transmitter idle.
- **FCR (16550A).** FCR0 enables the FIFOs, and any change of FCR0 clears
  both of them and the 16450-mode RBR/THR. Written with FCR0 = 1, FCR1 and
  FCR2 clear the RX and TX FIFO and D7:6 select the trigger level (1, 4, 8,
  14); with FCR0 = 0 they program nothing. A TX FIFO clear leaves the byte in
  the shifter to finish.
- **Character timeout.** With bytes in the RX FIFO and no byte received and
  no RBR read for four character times (4 × the frame's bits: start, data,
  parity, stop, with 1.5 stop bits counted as 2), IIR reports `0C` on the
  next bit tick. An RBR read clears it and restarts the timer; so does a
  received byte.
- **Transmit buffering.** `uart_tx_core` runs with `HOLD` = 0, so the
  transmitter holds what the data sheets give it: the THR (16450 mode) or
  the 16-byte TX FIFO, then the TSR. A waiting byte stays in the THR, or at
  the head of the FIFO, until the shifter takes it at its start bit, so THRE
  rises in the clock the last pending byte's start bit begins: THRE follows
  the start of transmission, as in the 8250's transmitter timing. In FIFO
  mode 16 bytes are accepted back to back, from idle or behind a frame in
  the shifter, and a write beyond them is dropped, unless the shifter takes
  the head in that clock, when the write takes the freed place. In 16450
  mode a THR write while THRE is low (driver misuse) replaces the waiting
  byte; one in the clock the shifter takes it waits behind it. A TX FIFO
  clear or an FCR0 change discards every waiting byte, in the clock the
  shifter would take one too, so only the byte in the shifter is sent. Word
  length and parity are latched as each byte enters the shifter, so at or
  before THRE's rise; the stop-bit count is sampled at the frame's last data
  bit, so a frame in flight keeps its whole format only if LCR changes after
  TEMT.
- **Modem status.** MSR D7:4 are the levels (DCD, RI, DSR, CTS) and D3:0 the
  deltas (DDCD, TERI on RI's trailing edge, DDSR, DCTS), cleared by an MSR
  read; a delta latched in the clock of the read survives it. The deltas do
  not latch for the first three clocks after reset, while the synchronizers
  fill, so a line held asserted through reset is not reported as a change.
- **Loopback** (MCR D4). The transmitter feeds the receiver internally, break
  included; TXD holds marking and RTS#/DTR# are forced inactive; MSR D7:4
  read `{OUT2, OUT1, DTR, RTS}`, with deltas from MCR writes, and the pins are
  ignored.
- **Break** (LCR D6) forces TXD spacing; the transmitter keeps shifting
  underneath it.

The serializer emits whole stop-bit periods. With a five-bit word, LCR D2
therefore emits two stop periods rather than the original part's 1.5; the
character-timeout and THRE-delay counters use the same whole-period count.
This can lengthen back-to-back five-bit frames, while the receiver still
samples the first stop bit.

## 8. Resource and timing character

All eight modules are small, flop-and-LUT designs with no memories beyond
the FIFO arrays (`DEPTH` bytes each in `usb_serial_fifo`; in `uart16550` two
16-byte arrays and a 16 × 3 error array, present only when `VARIANT` = 3),
inferred as distributed RAM or flops. The
longest combinational paths are the DDS accumulator's 33-bit add-compare and
the FIFO's pointer compares; there are no multi-cycle or false paths inside
any module. The asynchronous inputs and clock crossings use two-flop
synchronizers: RXD (section 3.2), USB byte-channel pointers and sidebands
(sections 6.2 and 6.3), and the 16550 modem inputs (section 7.1).
The consuming design supplies the external CDC constraints described there.

## 9. Verification

<a id="91-the-benches"></a>
### 9.1 Simulation

`sim/build.ninja` is the gate driver: `ninja` runs Verible syntax, lint and
format checks on every RTL and bench file; `ninja bench` runs eight
self-checking iverilog benches, each of which must end in `ALL CHECKS PASS`:

| Bench | What it proves |
|---|---|
| `tb_usb_serial_chan` (shared clock, and `-DASYNC_MODE` with unrelated clocks) | the byte loop both ways through a stack echo pump, TX and RX backpressure with the FIFO contents checked, the three sidebands, a mid-run reset |
| `tb_acia6551_to_usbserial` | the adapter on a shared-clock `usb_serial_chan` between a model of the ACIA's FIFOs and the stack window: a single TX byte popped in the cycle the channel takes it; a 20-byte burst into the 16-byte FIFO with the pop stopping exactly at full and every byte in order; an RX byte held while `rx_full_i` and written when room appears, then a run in order; `{RTS, DTR}` asserted and BAUD_PENDING never set |
| `tb_uart_tx_core` | every word length, parity mode and stop count decoded bit for bit at mid-bit; the holding buffer, back-to-back start timing, a refused write; the config latched at load; break; reset mid-frame; the state register forced to each unused code; a second instance without the buffer (`HOLD` = 0): `ready_o` only in the load clock, a `we_i` outside it ignored, back-to-back frames |
| `tb_uart_rx_core` | the same frame matrix; every parity, framing and break flag combination; glitch rejection; the line still low after a framing error; a 4/5-clock fractional tick grid; reset mid-frame; the forced state codes |
| `tb_uart_baud_dds` | exact average rates at 48 MHz, 200 MHz, 3.579545 MHz and `CLK_HZ = REF_HZ`; divisor 0 from reset and disabling at all sixteen bit phases; the full 16-bit divisor; resync; a divisor lowered mid-count; reset |
| `tb_uart_loop` | the three cores as a looped-back UART at 351.5625 clocks per bit, every frame config, three divisors |
| `tb_uart16550` | `uart16550` as all four variants plus a `RESET_DIVISOR` build, on one register bus with the bench driving RXD (bad parity, framing errors, break) and decoding TXD, against 8250/16450/16550A data-sheet behavior: reset values, every register bit, DLAB, chip select; on each variant 16450-mode receive, PE/FE/BI, sticky flags, overrun, reads aimed at the clock a byte lands, THRE timed to the start bit and TEMT to the frame, a byte behind a shifting frame, a THR write while THRE is low replacing the waiting byte (also in the clock the shifter takes it); the 16550A's FIFO clears (discarding every waiting byte, also in the clock the shifter would take one, and on an FCR0 change in 16450 mode), four trigger levels, character-timeout timing at three frame sizes, the 16 transmit places filled from idle and behind a shifting frame with the 17th write dropped and a write aimed at the clock the shifter takes the head, RX overrun and a byte completing into a full FIFO in the clock of an RBR read, errors revealed at the top of the FIFO, FIFOE across the whole FIFO (an error behind a clean head, an error landing in the clock of an LSR read, an RX clear, 16450 mode), every FIFO slot through every bit; IIR priority across all five sources and each IER mask, the THRE rules, the THRE-interrupt hold-back (a lone byte at two frame sizes, two bytes queued, the first after an FCR0 change, a THR write cancelling it, an ETBEI write or a TX FIFO clear during it); TXD frames in five formats, break, loopback, modem deltas with TERI, `int_o` gating; baud tick periods; reset mid-activity |

The pre-commit hook runs the gate and the benches. The `bench-checks` target
also compiles each `check` task in isolation and verifies that only known-one
conditions pass; zero, X and Z each count as failures.
It also checks the channel producer against ready rising after a sampling
edge, so the driver cannot release valid before the byte is accepted.
The 16550 range oracle rejects unknown measurements, and the transmitter idle
monitor is exercised with an unintended pulse inside its observation window.

### 9.2 Coverage

`sim/vl/cov.ps1` re-elaborates the same benches under Verilator with line,
branch and toggle instrumentation and merges the results;
`tools/cov_summary.py` renders [the committed baseline](coverage.md), which
owns the measured figures and hole triage. Its triage distinguishes literal
constants from invariants established by the proofs below.

<a id="93-the-invariant-proof"></a>
### 9.3 Formal verification

The [formal coverage map](formal-coverage.md) owns the parameter matrix,
assumptions, retained property counts and remaining gaps. `ninja formal` runs
core invariants, FIFO ordering and coordinated flush across the listed depths,
channel/ACIA composition and sidebands, RX sample/wire models, TX frame-bit
checks for both holding-buffer modes, exact baud spacing at selected ratios,
and register/modem/interrupt timing across 8250, 16450, 16550 and 16550A. `ninja formal-negative` also runs the declared
false-property controls. These optional checks use the shared audited runner;
RTL and bound properties are elaborated with Yosys/slang.

The coverage map specifies the RX jitter envelope, clock/reset assumptions and
parameter limits. Physical CDC qualification remains a consumer obligation.
Simulation coverage does not establish formal property coverage.
