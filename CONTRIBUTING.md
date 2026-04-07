# Contributing to PLFM_RADAR (AERIS-10)

Thanks for your interest in the project! This guide covers the basics
for getting a change reviewed and merged.

## Getting started

1. Fork the repository and create a topic branch from `develop`.
2. Keep generated outputs (Vivado projects, bitstreams, build logs)
   out of version control — the `.gitignore` already covers most of
   these.

## Repository layout

| Path | Contents |
|------|----------|
| `4_Schematics and Boards Layout/` | KiCad schematics, Gerbers, BOM/CPL |
| `9_Firmware/9_2_FPGA/` | Verilog RTL, constraints, testbenches, build scripts |
| `9_Firmware/9_2_FPGA/formal/` | SymbiYosys formal-verification wrappers |
| `9_Firmware/9_2_FPGA/scripts/` | Vivado TCL build & debug scripts |
| `9_Firmware/9_3_GUI/` | Python radar dashboard (Tkinter + matplotlib) |
| `docs/` | GitHub Pages documentation site |

## Before submitting a pull request

- **Python** — verify syntax: `python3 -m py_compile <file>`
- **Verilog** — if you have Vivado, run the relevant `build*.tcl`;
  if not, note which scripts your change affects
- **Whitespace** — `git diff --check` should be clean
- Keep PRs focused: one logical change per PR is easier to review
- **Run the regression tests** (see below)

## Running regression tests

After any change, run the relevant test suites to verify nothing is
broken. All commands assume you are at the repository root.

### Prerequisites

| Tool | Used by | Install |
|------|---------|---------|
| [Icarus Verilog](http://iverilog.icarus.com/) (`iverilog`) | FPGA regression | `brew install icarus-verilog` / `apt install iverilog` |
| Python 3.10+ | GUI tests, co-sim | Usually pre-installed |
| [uv](https://docs.astral.sh/uv/) | Python dependency sync, lint/test runner | `curl -LsSf https://astral.sh/uv/install.sh \| sh` |
| GNU Make | MCU tests | Usually pre-installed |
| [SymbiYosys](https://symbiyosys.readthedocs.io/) (`sby`) | Formal verification | Optional — see SymbiYosys docs |

### FPGA regression (RTL lint + unit/integration/signal-processing tests)

```bash
cd 9_Firmware/9_2_FPGA
bash run_regression.sh
```

This runs four phases:

| Phase | What it checks |
|-------|----------------|
| 0 — Lint | `iverilog -Wall` on all production RTL + static regex checks |
| 1 — Changed Modules | Unit tests for individual blocks (CIC, Doppler, CFAR, etc.) |
| 2 — Integration | DDC chain, receiver golden-compare, system-top, end-to-end |
| 3 — Signal Processing | FFT engine, NCO, FIR, matched filter chain |
| 4 — Infrastructure | CDC modules, edge detector, USB interface, range-bin decimator, mode controller |

All tests must pass (exit code 0). Advisory lint warnings (e.g., `case
without default`) are non-blocking.

### MCU unit tests

```bash
cd 9_Firmware/9_1_Microcontroller/tests
make clean && make all
```

Runs 20 C-based unit tests covering safety, bug-fix, and gap-3 tests.
Every test binary must exit 0.

### GUI / dashboard tests

```bash
uv sync --group dev
uv run pytest 9_Firmware/9_3_GUI/test_radar_dashboard.py -v

cd 9_Firmware/9_3_GUI
python3 -m unittest test_radar_dashboard -v
```

57+ protocol and rendering tests. The `test_record_and_stop` test
requires `h5py` and will be skipped if it is not installed.

### Co-simulation (Python vs RTL golden comparison)

Run from the co-sim directory after a successful FPGA regression (the
regression generates the RTL CSV outputs that the co-sim scripts compare
against):

```bash
cd 9_Firmware/9_2_FPGA/tb/cosim

# Validate all .mem files (twiddles, chirp ROMs, addressing)
python3 validate_mem_files.py

# DDC chain: RTL vs Python model (5 scenarios)
python3 compare.py dc
python3 compare.py single_target
python3 compare.py multi_target
python3 compare.py noise_only
python3 compare.py sine_1mhz

# Doppler processor: RTL vs golden reference
python3 compare_doppler.py stationary

# Matched filter: RTL vs Python model (4 scenarios)
python3 compare_mf.py all
```

Each script prints PASS/FAIL per scenario and exits non-zero on failure.

### Formal verification (optional)

Requires SymbiYosys (`sby`), Yosys, and a solver (z3 or boolector):

```bash
cd 9_Firmware/9_2_FPGA/formal
sby -f fv_doppler_processor.sby
sby -f fv_radar_mode_controller.sby
```

### Quick checklist

Before pushing, confirm:

1. `bash run_regression.sh` — all phases pass
2. `make all` (MCU tests) — 20/20 pass
3. `python3 -m unittest test_radar_dashboard -v` — all pass
4. `python3 validate_mem_files.py` — all checks pass
5. `python3 compare.py dc && python3 compare_doppler.py stationary && python3 compare_mf.py all`
6. `git diff --check` — no whitespace issues

## CI checks

GitHub Actions runs automatically on pull requests and pushes to `main` / `develop`:

- Python lint (`ruff`) + Python compile check
- Dashboard tests (`pytest`)
- FPGA regression (`bash run_regression.sh`)
- MCU unit tests (`make clean && make all`)

## Areas where help is especially welcome

See the list in [README.md](README.md#-contributing).

## Questions?

Open a GitHub issue — that way the discussion is visible to everyone.
