#!/usr/bin/env python3
"""
AERIS-10 UART Diagnostic Capture Tool

Captures STM32 DIAG output from USART3 (115200 8N1) and writes to both
the terminal and a timestamped log file. Designed for board-day bring-up.

DIAG output format (from diag_log.h):
    [  12345 ms] SUBSYS: message
    [  12345 ms] SUBSYS WARN: message
    [  12345 ms] SUBSYS **ERR**: message
    [  12345 ms] ======== Section Title ========

Subsystem tags: CLK, LO, LO_DRV, BF, PA, FPGA, USB, PWR, IMU, MOT, SYS

Requirements:
    pip install pyserial

Usage:
    python3 uart_capture.py                     # auto-detect port
    python3 uart_capture.py -p /dev/cu.usbmodem*  # explicit port
    python3 uart_capture.py --filter LO,PA      # only show LO and PA lines
    python3 uart_capture.py --errors-only       # only show WARN and ERR lines
    python3 uart_capture.py --no-log            # terminal only, no log file
    python3 uart_capture.py --list              # list available serial ports
"""

import argparse
import datetime
import glob
import os
import re
import signal
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_BAUD = 115200
ENCODING = "utf-8"
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs")

# ANSI color codes for terminal
COLORS = {
    "RESET": "\033[0m",
    "RED": "\033[91m",
    "YELLOW": "\033[93m",
    "GREEN": "\033[92m",
    "CYAN": "\033[96m",
    "DIM": "\033[2m",
    "BOLD": "\033[1m",
    "MAGENTA": "\033[95m",
}

# Subsystem tag → color mapping
SUBSYS_COLORS = {
    "CLK": "CYAN",
    "LO": "GREEN",
    "LO_DRV": "GREEN",
    "BF": "MAGENTA",
    "PA": "YELLOW",
    "FPGA": "CYAN",
    "USB": "CYAN",
    "PWR": "RED",
    "IMU": "DIM",
    "MOT": "DIM",
    "SYS": "BOLD",
}

# Regex patterns for DIAG output parsing
RE_DIAG_LINE = re.compile(
    r"^\[\s*(\d+)\s*ms\]\s+"  # timestamp
    r"(?:={8}\s+(.+?)\s+={8}|"  # section separator
    r"(\w+)"  # subsystem tag
    r"(?:\s+(WARN|\*\*ERR\*\*))?"  # optional severity
    r":\s+(.*))"  # message
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def list_ports():
    """Print available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print(f"{'Port':<30} {'Description':<40} {'HWID'}")
    print("-" * 100)
    for p in sorted(ports, key=lambda x: x.device):
        print(f"{p.device:<30} {p.description:<40} {p.hwid}")


def auto_detect_port():
    """Try to auto-detect the STM32 serial port on macOS."""
    # Priority order: ST-Link VCP, generic USB serial
    patterns = [
        "/dev/cu.usbmodem*",  # ST-Link Virtual COM Port
        "/dev/cu.usbserial*",  # FTDI/CH340/CP210x adapters
        "/dev/cu.SLAB*",  # Silicon Labs CP210x
        "/dev/cu.wchusbserial*",  # CH340/CH341
    ]
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def colorize(line, use_color=True):
    """Apply ANSI colors to a DIAG line for terminal display."""
    if not use_color:
        return line

    m = RE_DIAG_LINE.match(line)
    if not m:
        # Non-DIAG line — show dimmed
        return f"{COLORS['DIM']}{line}{COLORS['RESET']}"

    timestamp, section, subsys, severity, msg = m.groups()

    if section:
        # Section separator
        return (
            f"{COLORS['DIM']}[{timestamp:>7} ms]{COLORS['RESET']} "
            f"{COLORS['BOLD']}======== {section} ========{COLORS['RESET']}"
        )

    # Pick color based on severity first, then subsystem
    if severity == "**ERR**":
        color = COLORS["RED"]
        sev_str = f" {COLORS['RED']}**ERR**{COLORS['RESET']}"
    elif severity == "WARN":
        color = COLORS["YELLOW"]
        sev_str = f" {COLORS['YELLOW']}WARN{COLORS['RESET']}"
    else:
        color = COLORS.get(SUBSYS_COLORS.get(subsys, ""), "")
        sev_str = ""

    return (
        f"{COLORS['DIM']}[{timestamp:>7} ms]{COLORS['RESET']} "
        f"{color}{subsys}{COLORS['RESET']}{sev_str}: {msg}"
    )


def should_display(line, filter_subsys=None, errors_only=False):
    """Decide whether to display a line based on filters."""
    m = RE_DIAG_LINE.match(line)
    if not m:
        # Non-DIAG lines: always show (could be raw HAL_UART_Transmit output)
        return True

    _, section, subsys, severity, _ = m.groups()

    # Section separators always shown
    if section:
        return True

    # Error filter
    if errors_only and severity not in ("WARN", "**ERR**"):
        return False

    # Subsystem filter
    if filter_subsys and subsys not in filter_subsys:
        return False

    return True


# ---------------------------------------------------------------------------
# Stats tracker
# ---------------------------------------------------------------------------


class CaptureStats:
    """Track line counts per subsystem and severity."""

    def __init__(self):
        self.total = 0
        self.errors = 0
        self.warnings = 0
        self.by_subsys = {}
        self.start_time = time.time()

    def update(self, line):
        self.total += 1
        m = RE_DIAG_LINE.match(line)
        if not m:
            return
        _, section, subsys, severity, _ = m.groups()
        if section:
            return
        if subsys:
            self.by_subsys[subsys] = self.by_subsys.get(subsys, 0) + 1
        if severity == "**ERR**":
            self.errors += 1
        elif severity == "WARN":
            self.warnings += 1

    def summary(self):
        elapsed = time.time() - self.start_time
        lines = [
            "",
            "--- Capture Summary ---",
            f"Duration:  {elapsed:.1f}s",
            f"Lines:     {self.total}",
            f"Errors:    {self.errors}",
            f"Warnings:  {self.warnings}",
        ]
        if self.by_subsys:
            lines.append("By subsystem:")
            for tag in sorted(self.by_subsys, key=self.by_subsys.get, reverse=True):
                lines.append(f"  {tag:<8} {self.by_subsys[tag]}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main capture loop
# ---------------------------------------------------------------------------


def capture(port, baud, log_file, filter_subsys, errors_only, use_color):
    """Open serial port and capture DIAG output."""
    stats = CaptureStats()
    running = True

    def handle_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,  # 100ms read timeout for responsive Ctrl-C
        )
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

    print(f"Connected to {port} at {baud} baud")
    if log_file:
        print(f"Logging to {log_file}")
    if filter_subsys:
        print(f"Filter: {', '.join(sorted(filter_subsys))}")
    if errors_only:
        print("Mode: errors/warnings only")
    print("Press Ctrl-C to stop.\n")

    flog = None
    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        flog = open(log_file, "w", encoding=ENCODING)
        flog.write(f"# AERIS-10 UART capture — {datetime.datetime.now().isoformat()}\n")
        flog.write(f"# Port: {port}  Baud: {baud}\n")
        flog.write(f"# Host: {os.uname().nodename}\n\n")
        flog.flush()

    line_buf = b""

    try:
        while running:
            try:
                chunk = ser.read(256)
            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                break

            if not chunk:
                continue

            line_buf += chunk

            # Process complete lines
            while b"\n" in line_buf:
                raw_line, line_buf = line_buf.split(b"\n", 1)
                line = raw_line.decode(ENCODING, errors="replace").rstrip("\r")

                if not line:
                    continue

                stats.update(line)

                # Log file always gets everything (unfiltered, no color)
                if flog:
                    wall_ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    flog.write(f"{wall_ts}  {line}\n")
                    flog.flush()

                # Terminal display respects filters
                if should_display(line, filter_subsys, errors_only):
                    print(colorize(line, use_color))

    finally:
        ser.close()
        if flog:
            flog.write(f"\n{stats.summary()}\n")
            flog.close()
        print(stats.summary())


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="AERIS-10 UART Diagnostic Capture Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage:")[0],
    )
    parser.add_argument(
        "-p",
        "--port",
        help="Serial port (default: auto-detect)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--filter",
        help="Comma-separated subsystem tags to display (e.g. LO,PA,CLK)",
    )
    parser.add_argument(
        "--errors-only",
        action="store_true",
        help="Only display WARN and ERR lines",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable log file output",
    )
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable ANSI color output",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available serial ports and exit",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Log file path (default: logs/uart_YYYYMMDD_HHMMSS.log)",
    )

    args = parser.parse_args()

    if args.list:
        list_ports()
        sys.exit(0)

    # Resolve port
    port = args.port
    if not port:
        port = auto_detect_port()
        if not port:
            print("ERROR: No serial port detected. Use -p to specify, or --list to see ports.")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    # Resolve log file
    log_file = None
    if not args.no_log:
        if args.output:
            log_file = args.output
        else:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = os.path.join(LOG_DIR, f"uart_{ts}.log")

    # Parse filter
    filter_subsys = None
    if args.filter:
        filter_subsys = set(t.strip().upper() for t in args.filter.split(","))

    # Color detection
    use_color = not args.no_color and sys.stdout.isatty()

    capture(port, args.baud, log_file, filter_subsys, args.errors_only, use_color)


if __name__ == "__main__":
    main()
