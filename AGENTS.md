<!-- SPDX-License-Identifier: MIT OR Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Howard M. Harte -->
# hh-uart: agent and contributor guide

Read [README.md](README.md) for setup and gate commands, and
[the design document](doc/hh-uart-design.md) for module interfaces and
verification scope. Humans and agents use the same documents. Every module, port and parameter name in
`rtl/` is public API that downstream designs instantiate directly: never
rename one for style.
