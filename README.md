# hh-uart

<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Howard M. Harte -->

Vendor-neutral SystemVerilog UART and USB-serial building blocks, including
an 8250/16450/16550/16550A register-compatible UART, serial RX/TX cores, a baud
generator, and a FIFO-backed byte-channel interface for a USB host stack.

## Documentation

| Document | Contents |
|---|---|
| [Design](doc/hh-uart-design.md) | Architecture, interfaces, integration rules, and verification scope. |
| [Coverage](doc/coverage.md) | Measured coverage and explanations of uncovered points. |
| [Formal coverage](doc/formal-coverage.md) | Proved properties, elaborated configurations, assumptions, audit results, and gaps. |
| [RTL style](https://github.com/s100box/retrosim-build/blob/main/doc/rtl-style.md) | Shared formatting, local lint exceptions, and checked scope. |

## Repository layout

| Path | Contents |
|---|---|
| `rtl/` | Reusable cores; module inventory in the design document. |
| `hh-uart.core` | FuseSoC file lists and targets. |
| `sim/` | Benches, lint rules, and optional coverage tools. |
| `tools/` | Coverage report generation. |
| `doc/` | Design and coverage references. |

## Requirements

### Tools

| Tool | Verified version | Used for |
|---|---|---|
| Python | 3.10 or later | shared hook checks and coverage/report tools |
| [Verible](https://github.com/chipsalliance/verible) | v0.0-4053 | syntax, lint and format check (`ninja`) |
| [Icarus Verilog](https://github.com/steveicarus/iverilog) (`iverilog`, `vvp`) from the [OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build) | 14.0 | the benches (`ninja bench`) |
| [Ninja](https://ninja-build.org/) | any | drives both |
| [Verilator](https://www.veripool.org/verilator/) from the OSS CAD Suite, with Visual Studio (MSVC) and its CMake | 5.049 | the coverage suite (`sim/vl/cov.ps1`), opt-in |
| [SymbiYosys](https://github.com/YosysHQ/sby) (`sby`, Yosys with the slang plugin, boolector) from the OSS CAD Suite | Yosys 0.65 | the contract proofs (`ninja formal`) |

### Sibling repositories

`sim/build.ninja` includes `../../retrosim-build/toolchain.ninja`, the
workspace's single file of tool install paths, so the checkout must sit next
to its sibling:

| Repository | Provides |
|---|---|
| `retrosim-build` | `toolchain.ninja`, which defines the Verible tool paths the gate uses; `hooks/gate-env.sh`, which the pre-commit hook sources to put Verible and the OSS CAD Suite on PATH; and `tools/vl_cov.ps1` + `tools/cov_summary_lib.py`, the coverage suite's driver and report renderer (which find Verilator and the MSVC build environment through `toolchain.ps1` and `tools/vs_env.ps1`) |

```text
<workspace>/
  hh-uart/
  retrosim-build/
```

Point `verible_root` in `retrosim-build/toolchain.ninja` at the Verible
release on your machine. The bench finds `iverilog` and `vvp` on PATH; the
pre-commit hook also looks for an OSS CAD Suite unpacked at
`$HOME/oss-cad-suite` or named by `OSS_CAD_SUITE`.

## Getting started

Compile the files listed in [hh-uart.core](hh-uart.core), or use its FuseSoC
targets. The RTL itself has no sibling dependencies. Integration examples,
clock-crossing constraints, and the register-compatible UART's documented
limitations are in the [design document](doc/hh-uart-design.md#7-building-a-uart-from-the-cores).

Module, port, and parameter names are public API. Update downstream users
together with any interface change.

## Verification

From the repository root:

```sh
ninja -C sim                  # syntax, lint, and format checks
ninja -C sim bench            # self-checking Icarus benches
ninja -C sim formal           # optional SymbiYosys contract proofs
ninja -C sim formal-negative  # include false-property controls
```

Individual targets are `bench-sync`, `bench-async`, `bench-acia`, `bench-tx`, `bench-rx`,
`bench-baud`, `bench-loop`, and `bench-16550`. The
[verification specification](doc/hh-uart-design.md#9-verification) describes
their coverage and the formal properties.

`bench-checks` additionally tests that the benches' assertion helpers reject
false, unknown, and high-impedance conditions; it also runs as part of `bench`.

### Coverage

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File sim/vl/cov.ps1
python tools/cov_summary.py
```

The optional suite writes annotated results under `sim/vl/build/cov/report/`
and regenerates [doc/coverage.md](doc/coverage.md). Refresh the baseline after
RTL or bench changes; `python tools/cov_summary.py --holes` lists uncovered points.

### Commit checks

Enable the hook with `git config core.hooksPath .githooks`. It runs lint,
format checks, and simulation; the formal and coverage suites are separate.
Verible uses the default lint rules and the 100-column `.verible.format`.

## License

Licensed under either [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.

### Contributions

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.
