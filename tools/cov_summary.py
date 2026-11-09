#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# SPDX-FileCopyrightText: 2026 Howard M. Harte
"""cov_summary.py -- render doc/coverage.md from the merged Verilator coverage
.dat produced by sim/vl/cov.ps1.

Declares this repository's report (scope, notes, prose); the parser and the
layout are retrosim-build/tools/cov_summary_lib.py. The hand-authored
"## Hole triage" section of doc/coverage.md is kept across regenerations.

Usage:  python tools/cov_summary.py [--report-dir <dir>] [--out doc/coverage.md]
        python tools/cov_summary.py --holes      # also list uncovered points to stderr
"""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LIB = Path(os.environ.get("RETROSIM_BUILD", REPO.parent / "retrosim-build")) / "tools"
if not LIB.is_dir():
    sys.exit("cov_summary: missing ../retrosim-build sibling (check it out beside this repository)")
sys.path.insert(0, str(LIB))
import cov_summary_lib as cov  # noqa: E402

REPORT = cov.Report(
    repo=REPO,
    title="hh-uart RTL code coverage (Verilator)",
    body=[
        "- Suite (the benches `sim/vl/CMakeLists.txt` builds): the same "
        "self-checking benches `ninja bench` runs under iverilog, re-elaborated "
        "under Verilator with `--main`. Points are merged by logical identity "
        "across benches and instances (covered if any instance hit it).\n",
        "- Scope: coverage is instrumented on the synthesizable RTL only "
        "(`rtl/*.sv`); the testbenches are excluded by `sim/vl/coverage.vlt`.\n",
    ],
    sections=[cov.Section("RTL", cov.is_sv_under("rtl"))],
    notes={
        "acia6551_to_usbserial.sv": "combinational byte handshake and fixed sidebands",
        "uart16550.sv": "register file, FIFOs, IIR, modem; all four VARIANTs",
        "uart_rx_core.sv": "16x-oversampling receiver FSM",
        "uart_tx_core.sv": "transmitter FSM; both generate arms (HOLD 1/0)",
        "uart_baud_dds.sv": "DDS + /divisor + /16 tick generator",
        "usb_serial_if.sv": "interface: wires only (no logic to cover)",
        "usb_serial_fifo.sv": "both generate arms (SAME_CLOCK 0/1)",
        "usb_serial_chan.sv": "FIFO pair + sidebands + register window",
    },
)

if __name__ == "__main__":
    raise SystemExit(cov.main(REPORT))
