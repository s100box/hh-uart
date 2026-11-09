#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# SPDX-FileCopyrightText: 2026 Howard M. Harte
"""Compile actual bench tasks against four-state and handshake edge cases.

The task is copied into a minimal module so this oracle regression does not
need to run the UART traffic stimulus. A passing assertion must be known one;
zero, X and Z must each increment the bench's failure count.
The channel producer must also observe ready at a transfer edge rather than
mistaking a later ready transition for an accepted byte.
"""

from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


class BenchChecks(unittest.TestCase):
    def simulate(self, text, sources=()):
        compiler, runtime = shutil.which("iverilog"), shutil.which("vvp")
        self.assertIsNotNone(compiler, "iverilog is required")
        self.assertIsNotNone(runtime, "vvp is required")
        with tempfile.TemporaryDirectory() as directory:
            source, image = Path(directory) / "oracle.sv", Path(directory) / "oracle.vvp"
            source.write_text(text, encoding="utf-8")
            compiled = subprocess.run([compiler, "-g2012", "-s", "oracle", "-o", str(image),
                                       *(str(p) for p in sources), str(source)],
                                      capture_output=True, text=True, timeout=30)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([runtime, str(image)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return result.stdout

    def test_register_bench_boolean_and_range_checks_reject_unknowns(self):
        bench = Path(__file__).parent / "tb_uart16550.sv"
        text = bench.read_text(encoding="utf-8")
        tasks = []
        for name in ("chk", "chk_rng"):
            match = re.search(r"\btask\s+automatic\s+" + name + r"\b.*?\bendtask\b", text, re.DOTALL)
            self.assertIsNotNone(match, f"keep the {name} assertion helper covered")
            tasks.append(match.group())
        self.simulate("""
module oracle;
  integer errors = 0, checks = 0, cur = 0;
  logic [8*24-1:0] sect = "oracle";
""" + "\n".join(tasks) + """
  initial begin
    chk("true", 1'b1);
    if (errors !== 0) $fatal(1, "true was rejected");
    chk("false", 1'b0);
    chk("X", 1'bx);
    chk("Z", 1'bz);
    if (errors !== 3) $fatal(1, "boolean checker accepted false/X/Z");
    chk_rng("low boundary", 10, 10, 20);
    chk_rng("high boundary", 20, 10, 20);
    if (errors !== 3) $fatal(1, "inclusive boundaries were rejected");
    chk_rng("X", 32'bx, 0, 20);
    if (errors !== 4) $fatal(1, "range checker accepted X");
    chk_rng("Z", 32'bz, 0, 20);
    chk_rng("below", 9, 10, 20);
    chk_rng("above", 21, 10, 20);
    if (errors !== 7) $fatal(1, "range checker accepted Z or an out-of-range value");
    $finish;
  end
endmodule
""")

    def test_idle_monitor_detects_a_start_within_the_observation_window(self):
        text = (Path(__file__).parent / "tb_uart_tx_core.sv").read_text(encoding="utf-8")
        tasks = []
        for name in ("check", "check_marking"):
            match = re.search(r"\btask\s+automatic\s+" + name + r"\b.*?\bendtask\b", text, re.DOTALL)
            self.assertIsNotNone(match, f"keep the {name} task covered")
            tasks.append(match.group())
        self.simulate("""
module oracle;
  integer errors = 0;
  logic clk = 0, txd = 1;
  always #5 clk = ~clk;
""" + "\n".join(tasks) + """
  initial begin
    fork
      check_marking(8);
      begin
        repeat (3) @(negedge clk);
        txd <= 0;
        @(negedge clk);
        txd <= 1;
      end
    join
    if (errors != 1) $fatal(1, "monitor missed a pulse inside the idle interval");
    check_marking(8);
    if (errors != 1) $fatal(1, "monitor rejected an idle interval");
    $finish;
  end
endmodule
""")

    def test_host_driver_waits_for_an_actual_transfer_edge(self):
        bench = Path(__file__).parent / "tb_usb_serial_chan.sv"
        match = re.search(r"\btask\s+automatic\s+host_send\b.*?\bendtask\b",
                          bench.read_text(encoding="utf-8"), re.DOTALL)
        self.assertIsNotNone(match, "the channel driver task must remain covered")
        compiler, runtime = shutil.which("iverilog"), shutil.which("vvp")
        self.assertIsNotNone(compiler)
        self.assertIsNotNone(runtime)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "driver.sv"
            source.write_text("""
module oracle;
  logic host_clk = 0;
  always #5 host_clk = ~host_clk;
  usb_serial_if us ();
  integer accepted = 0;
  always @(posedge host_clk)
    if (us.tx_valid && us.tx_ready) begin
      accepted = accepted + 1;
      if (us.tx_data !== 8'hA5) $fatal(1, "driver changed the payload");
    end
""" + match.group() + """
  initial begin
    us.tx_valid = 0;
    us.tx_ready = 0;
    fork
      begin
        host_send(8'hA5);
        if (accepted != 1) $fatal(1, "driver returned without exactly one transfer");
      end
      begin
        wait (us.tx_valid);
        @(posedge host_clk);
        us.tx_ready <= 1;
        @(posedge host_clk);
        us.tx_ready <= 0;
      end
    join
    $finish;
  end
  initial begin
    #1000;
    $fatal(1, "driver did not return");
  end
endmodule
""", encoding="utf-8")
            image = root / "driver.vvp"
            interface = bench.parent.parent / "rtl" / "usb_serial_if.sv"
            compiled = subprocess.run([compiler, "-g2012", "-s", "oracle", "-o", str(image),
                                       str(interface), str(source)], capture_output=True, text=True, timeout=30)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            result = subprocess.run([runtime, str(image)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_false_and_unknown_conditions_are_failures(self):
        compiler = shutil.which("iverilog")
        runtime = shutil.which("vvp")
        self.assertIsNotNone(compiler, "iverilog is required for bench oracle tests")
        self.assertIsNotNone(runtime, "vvp is required for bench oracle tests")
        checked = 0
        for bench in sorted(Path(__file__).parent.glob("tb_*.sv")):
            match = re.search(r"\btask\s+automatic\s+check\b.*?\bendtask\b",
                              bench.read_text(encoding="utf-8"), re.DOTALL)
            if match is None:
                continue
            checked += 1
            with self.subTest(bench=bench.name), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                source = root / "oracle.sv"
                source.write_text(
                    "module oracle; integer errors = 0;\n" + match.group() + "\n" + """
  initial begin
    check(1'b1, "known true");
    if (errors !== 0) $fatal(1, "true was rejected");
    check(1'b0, "known false");
    if (errors !== 1) $fatal(1, "false was accepted");
    check(1'bx, "unknown");
    if (errors !== 2) $fatal(1, "X was accepted");
    check(1'bz, "high impedance");
    if (errors !== 3) $fatal(1, "Z was accepted");
    $display("ORACLE PASS");
    $finish;
  end
endmodule
""", encoding="utf-8")
                image = root / "oracle.vvp"
                compiled = subprocess.run([compiler, "-g2012", "-s", "oracle", "-o", str(image), str(source)],
                                          capture_output=True, text=True, timeout=30)
                self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
                result = subprocess.run([runtime, str(image)], capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("ORACLE PASS", result.stdout)
        self.assertEqual(checked, 6, "review new/removed assertion helpers in the bench inventory")


if __name__ == "__main__":
    unittest.main()
