# hh-uart formal verification coverage

<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Howard M. Harte -->

[README](../README.md) · [Design](hh-uart-design.md#93-formal-verification)

## 1. Scope

The optional suite establishes the digital contracts below. All positive safety
jobs use unbounded induction or PDR; their configured depths are not execution
limits. Simulation coverage is a separate measure. The consumer owns the
properties and parameter matrix; the shared runner audits fresh jobs, copied
source hashes, retained cells, verdicts and counterexamples.

## 2. Property and parameter matrix

| Family | Configuration | Contract |
|---|---|---|
| Core invariants | RX; TX `HOLD=0,1`; DDS `CLK_HZ=200000000`, `REF_HZ=1843200`, arbitrary divisor/resync | State and data-mask bounds, TX admission, DDS accumulator bounds, disabled-output suppression and held /16 phase |
| USB FIFO | Sync depths 2/4/8/16; async depths 4/8/16; arbitrary data, requests and independently paused clocks | Accepted-byte ordering, initialized reads, occupancy, conservative flags, non-wrapping historical pointer order and coordinated reset flush |
| Channel / ACIA | `DEPTH=4`, `SAME_CLOCK=0,1`, direct interface or real `acia6551_to_usbserial` adapter | Both directions' accepted-byte preservation through actual FIFOs; backpressure and pop/write strobes; register mux; baud capture/ack and modem synchronization/pending semantics |
| RX samples | All 5–8-bit lengths and parity modes; arbitrary line and sample ticks | Independent sample-age model: complete byte, raw parity slot, PE/FE/BI flags, glitch rejection and exact one-clock strobe |
| RX wire | Same frame matrix, symbolic payload and injected parity/stop errors; 16 core/sample clocks per bit, one frame per 256 clocks | End-to-end byte/flags and exactly-once delivery despite arbitrary line values at bit phases 15 and 0; phases 1–14 carry the specified bit |
| TX frame | `HOLD=0,1`; all frame formats, data and break inputs free | Start, LSB-first data, parity and complete stop periods; buffer preservation; ready/backpressure; late stop2 sampling and arbitrary tick pauses |
| Baud timing | Ratio-scaled `(CLK_HZ,REF_HZ,divisor)` = `(8,2,1)`, `(7,3,1)`, `(5,2,3)`, `(3,2,1)`; arbitrary resync/reset | Exact floor/ceiling core-clock intervals for sample and bit ticks; reference phase conservation; sixteen consumed sample ticks per bit except interrupted phases |
| UART registers | 8250/16450/16550A at `CLK_HZ=7372800`; 16550 at `11059200`; all `REF_HZ=1843200` | FIFO/error bookkeeping, DLAB/register preservation, exact LSR/MSR clear/set priorities, modem synchronization/edge flags, IIR priority/gating, THRE re-arm/delay and timeout tick timing |

## 3. Assumptions and interpretation

Single-clock jobs begin with synchronous reset; later reset is free. RX config
must remain stable while a frame is active, as required of the parent UART.
The sample-age model permits arbitrary line transitions and tick pauses; it
does not assume a successful frame. The wire model additionally holds its
symbolic payload/configuration/error selection stable and drives the explicit
periodic waveform above. It checks a finite digital jitter envelope, not a
general transmitter/receiver frequency-error budget or analog metastability.

TX length and parity latch at load; `stop2_i` is sampled at the final data bit.
Break overrides the pin while serialization continues. No fairness assumption
promises another tick or another producer/consumer transaction.

FIFO and channel drivers use a native Yosys global-clock wrapper; RTL and
properties use slang. Both domain clocks run during startup reset and may pause
independently afterward. FIFO `restart` holds a common flush while both domains
observe reset. Pending bytes are intentionally discarded on that flush. This
does not support a unilateral reset or promise delivery while a clock/consumer
stays paused. Memory cells are not cleared; initialized-read checks restart.
Supported depths are powers of two: sync at least two, async at least four.
Only the listed elaborations are proved, not every legal depth or data width.

Channel proofs instantiate the actual channel, FIFOs and optional ACIA adapter;
external transaction scoreboards check each accepted byte. A full DATA_RX write
is dropped; an empty DATA_TX read does not consume a byte. Sidebands are
unbuffered digital crossings, not event queues: rapid baud toggles may coalesce
or cancel, and a capture reads the currently held source word. Atomic delivery
requires source-word stability until capture and receiver clocks that run.
Independent byte reads are not an atomic snapshot during another capture.
Physical multi-bit settling, Gray-bus skew and synchronizer resolution remain
consumer timing obligations.

Baud interval checks use constant divisors at the listed ratios. Dynamic
divisor changes and disable retain the baseline bounds and simulation checks;
there is no fixed-spacing promise across a rate change. Resync resets the /16
sample-token count while the sample divider remains free-running. Its first
bit tick follows sixteen subsequently consumed sample pulses, rather than a
newly aligned sample period measured from the resync edge.

Register inputs remain free after reset, including simultaneous CPU, RX and
modem events. New LSR/MSR errors win over a same-cycle read acknowledgement.
Timeout counting restarts on RX, RBR read or clear, and fires on the bit tick
observing the already-reached four-character threshold. FIFO clear, THR write,
IIR acknowledgement and ETBEI re-arm priorities are checked explicitly. These
are the documented digital implementation contracts, not full silicon errata
or an arbitrary mid-frame LCR-change compatibility claim.

## 4. Audit and reproduction

```sh
ninja -C sim formal
ninja -C sim formal-negative
```

Focused targets include `formal-rx`, `formal-channel`, `formal-baud`,
`formal-fifo`, `formal-frame` and `formal-16550`. Use the documented Yosys 0.65,
slang, SymbiYosys, ABC and Boolector toolchain. Fresh work directories under
ignored `sim/formal/build/` prevent reuse of stale verdicts. `audit.json`
records the source SHA-256 values and per-job assertion/assumption/cover census.
Counts describe this frontend/model, not a percentage of correctness.
Manifest assertion/cover floors retain the audited census, so losing properties
requires an explicit matrix review rather than passing a merely nonempty model.

| Positive safety job | Assert | Assume |
|---|---:|---:|
| Core invariants | 13 | 1 |
| FIFO sync depth 2/4/8/16 | 11/15/23/39 | 0 |
| FIFO async depth 4/8/16 | 35/43/59 | 0 |
| Channel direct sync/async | 50/78 | 0 |
| Channel with ACIA sync/async | 46/77 | 0 |
| RX samples | 17 | 2 |
| RX wire | 49 | 3 |
| TX direct/buffered | 6/8 | 1 |
| Each baud timing case | 20 | 1 |
| 8250/16450/16550 registers | 66 each | 1 each |
| 16550A registers | 68 | 1 |

Cover jobs reach full/drain/wrap and reset flush, both channel directions,
sideband acknowledgements, frame/error delivery, rejected glitches, repeated
baud ticks and rephasing, and register/IRQ activity. The core-invariant baseline
has no covers. Each expanded family includes a false-property job that must
return FAIL with a VCD counterexample; ERROR, UNKNOWN, timeout or a missing
trace fails the audit. Positive proofs and covers must pass too.

## 5. Limits

This matrix does not quantify over all legal parameters, asynchronous unilateral
resets, every runtime rate/configuration transition, or every physical RX noise
and frequency-error condition. Those limits do not replace the Icarus,
coverage, CDC/timing and hardware gates.
