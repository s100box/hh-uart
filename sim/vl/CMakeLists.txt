# SPDX-License-Identifier: MIT OR Apache-2.0
# SPDX-FileCopyrightText: 2026 Howard M. Harte
cmake_minimum_required(VERSION 3.20)
project(hh_uart_vl CXX)

# --timing uses C++20 coroutines (verilated_timing.cpp) -> require C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(verilator REQUIRED)
include(${CMAKE_CURRENT_LIST_DIR}/../../../retrosim-build/tools/verilator.cmake)

# ---- Opt-in Verilator code-coverage instrumentation -------------------------
# VL_COVERAGE=ON appends --coverage-line + --coverage-toggle (+ the coverage.vlt
# focus filter) to EVERY retrosim_verilate() target, so the whole bench set can be re-run
# instrumented and merged with verilator_coverage. Default OFF. The coverage
# twins build into a SEPARATE dir (build/cov/) via vl/cov.ps1. Code coverage only
# -- SV covergroups are unsupported by Verilator. The two sub-flags rather than
# the --coverage alias: --coverage also turns on --coverage-user (SVA `cover`),
# and the benches author none.
option(VL_COVERAGE "Instrument the verilated models for code coverage" OFF)
set(VL_COV_ARGS "")
if(VL_COVERAGE)
  set(VL_COV_ARGS --coverage-line --coverage-toggle ${CMAKE_CURRENT_SOURCE_DIR}/coverage.vlt)
endif()

set(RTL ${CMAKE_CURRENT_SOURCE_DIR}/../../rtl)
set(SIM ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Shared verilate args. --main makes Verilator emit the C++ main for these
# self-checking SV benches (their own initial blocks drive clk + $finish), so no
# hand-written sim_main.cpp is needed; empty_main.cpp only satisfies
# add_executable()'s require-a-source rule.
set(VL_COMMON
  --main --timing -DSIM --trace-depth 0
  -Wno-WIDTH -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC
  -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME
  -Wno-MULTITOP -Wno-fatal
  --x-assign 0 --x-initial 0
)

# bench(name top [DEFINES <-Dfoo...>] SOURCES ...) -> a vl-<name> executable.
function(bench name top)
  cmake_parse_arguments(B "" "" "DEFINES;SOURCES" ${ARGN})
  add_executable(vl-${name} empty_main.cpp)
  retrosim_verilate(vl-${name}
    TOP_MODULE ${top}
    THREADS 1
    PREFIX V_${name}
    SOURCES ${B_SOURCES}
    VERILATOR_ARGS ${VL_COMMON} ${B_DEFINES} ${VL_COV_ARGS}
  )
endfunction()

# ---- The bench set: the same benches `ninja bench` runs under iverilog -------
set(CHAN ${RTL}/usb_serial_if.sv ${RTL}/usb_serial_fifo.sv ${RTL}/usb_serial_chan.sv)

# usb_serial_chan + the FIFO under it, both clocking builds (SAME_CLOCK selects
# two different FIFO generate arms; only one of them is a CDC).
bench(uss_sync  tb_usb_serial_chan SOURCES ${CHAN} ${SIM}/tb_usb_serial_chan.sv)
bench(uss_async tb_usb_serial_chan DEFINES -DASYNC_MODE SOURCES ${CHAN} ${SIM}/tb_usb_serial_chan.sv)

# Keep the adapter bench in parity with the Icarus gate and report scope.
bench(acia6551 tb_acia6551_to_usbserial SOURCES ${CHAN} ${RTL}/acia6551_to_usbserial.sv ${SIM}/tb_acia6551_to_usbserial.sv)

# The UART cores, each alone, then the three together as a looped-back UART.
bench(uart_tx_core  tb_uart_tx_core  SOURCES ${RTL}/uart_tx_core.sv  ${SIM}/tb_uart_tx_core.sv)
bench(uart_rx_core  tb_uart_rx_core  SOURCES ${RTL}/uart_rx_core.sv  ${SIM}/tb_uart_rx_core.sv)
bench(uart_baud_dds tb_uart_baud_dds SOURCES ${RTL}/uart_baud_dds.sv ${SIM}/tb_uart_baud_dds.sv)
bench(uart_loop     tb_uart_loop     SOURCES ${RTL}/uart_baud_dds.sv ${RTL}/uart_tx_core.sv ${RTL}/uart_rx_core.sv ${SIM}/tb_uart_loop.sv)
bench(uart16550     tb_uart16550     SOURCES ${RTL}/uart_baud_dds.sv ${RTL}/uart_tx_core.sv ${RTL}/uart_rx_core.sv ${RTL}/uart16550.sv ${SIM}/tb_uart16550.sv)
