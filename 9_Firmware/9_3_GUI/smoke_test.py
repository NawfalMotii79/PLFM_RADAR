#!/usr/bin/env python3
"""
AERIS-10 Board Bring-Up Smoke Test — Host-Side Script
======================================================
Sends opcode 0x30 to trigger the FPGA self-test, then reads back
the results via opcode 0x31. Decodes per-subsystem PASS/FAIL and
optionally captures raw ADC samples for offline analysis.

Usage:
  python smoke_test.py              # Mock mode (no hardware)
  python smoke_test.py --live       # Real FT2232H hardware
  python smoke_test.py --live --adc-dump adc_raw.npy  # Capture ADC data

Self-Test Subsystems:
  Bit 0: BRAM write/read pattern (walking 1s)
  Bit 1: CIC integrator arithmetic
  Bit 2: FFT butterfly arithmetic
  Bit 3: Saturating add (MTI-style)
  Bit 4: ADC raw data capture (256 samples)

Exit codes:
  0 = all tests passed
  1 = one or more tests failed
  2 = communication error / timeout
"""

import sys
import os
import time
import argparse
import logging

import numpy as np

# Add parent directory for radar_protocol import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radar_protocol import RadarProtocol, FT2232HConnection

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("smoke_test")

# Self-test opcodes (must match radar_system_top.v command decode)
OPCODE_SELF_TEST_TRIGGER = 0x30
OPCODE_SELF_TEST_RESULT  = 0x31

# Result packet format (sent by FPGA after self-test completes):
# The self-test result is reported via the status readback mechanism.
# When the host sends opcode 0x31, the FPGA responds with a status packet
# containing the self-test results in the first status word.
#
# For mock mode, we simulate this directly.

TEST_NAMES = {
    0: "BRAM Write/Read Pattern",
    1: "CIC Integrator Arithmetic",
    2: "FFT Butterfly Arithmetic",
    3: "Saturating Add (MTI)",
    4: "ADC Raw Data Capture",
}


class SmokeTest:
    """Host-side smoke test controller."""

    def __init__(self, connection: FT2232HConnection, adc_dump_path: str | None = None):
        self.conn = connection
        self.adc_dump_path = adc_dump_path
        self._adc_samples = []

    def run(self) -> bool:
        """
        Execute the full smoke test sequence.
        Returns True if all tests pass, False otherwise.
        """
        log.info("=" * 60)
        log.info("  AERIS-10 Board Bring-Up Smoke Test")
        log.info("=" * 60)
        log.info("")

        # Step 1: Connect
        if not self.conn.is_open and not self.conn.open():
            log.error("Failed to open FT2232H connection")
            return False

        # Step 2: Send self-test trigger (opcode 0x30)
        log.info("Sending self-test trigger (opcode 0x30)...")
        cmd = RadarProtocol.build_command(OPCODE_SELF_TEST_TRIGGER, 1)
        if not self.conn.write(cmd):
            log.error("Failed to send trigger command")
            return False

        # Step 3: Wait for completion and read results
        log.info("Waiting for self-test completion...")
        result = self._wait_for_result(timeout_s=5.0)

        if result is None:
            log.error("Timeout waiting for self-test results")
            return False

        # Step 4: Decode results
        result_flags, result_detail = result
        all_pass = self._decode_results(result_flags, result_detail)

        # Step 5: ADC data dump (if requested and test 4 passed)
        if self.adc_dump_path and (result_flags & 0x10):
            self._save_adc_dump()

        # Step 6: Summary
        log.info("")
        log.info("=" * 60)
        if all_pass:
            log.info("  SMOKE TEST: ALL PASS")
        else:
            log.info("  SMOKE TEST: FAILED")
        log.info("=" * 60)

        return all_pass

    def _wait_for_result(self, timeout_s: float):
        """
        Poll for self-test result.
        Returns (result_flags, result_detail) or None on timeout.
        """
        if self.conn._mock:
            # Mock: simulate successful self-test after a short delay
            time.sleep(0.2)
            return (0x1F, 0x00)  # All 5 tests pass

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            # Request result readback (opcode 0x31)
            cmd = RadarProtocol.build_command(OPCODE_SELF_TEST_RESULT, 0)
            self.conn.write(cmd)
            time.sleep(0.1)

            # Read response
            raw = self.conn.read(256)
            if raw is None:
                continue

            # Look for status packet (0xBB header)
            packets = RadarProtocol.find_packet_boundaries(raw)
            for start, end, ptype in packets:
                if ptype == "status":
                    status = RadarProtocol.parse_status_packet(raw[start:end])
                    if status is not None:
                        # Self-test results live in dedicated status fields
                        # (word 5: self_test_flags[4:0], self_test_detail[7:0]).
                        # See radar_protocol.py:257 and test_GUI_V65_Tk.py:198.
                        result_flags = status.self_test_flags
                        result_detail = status.self_test_detail
                        return (result_flags, result_detail)

            time.sleep(0.1)

        return None

    def _decode_results(self, flags: int, detail: int) -> bool:
        """Decode and display per-test results. Returns True if all pass."""
        log.info("")
        log.info("Self-Test Results:")
        log.info("-" * 40)

        all_pass = True
        for bit, name in TEST_NAMES.items():
            passed = bool(flags & (1 << bit))
            status = "PASS" if passed else "FAIL"
            marker = "✓" if passed else "✗"
            log.info(f"  {marker} Test {bit}: {name:30s} [{status}]")
            if not passed:
                all_pass = False

        log.info("-" * 40)
        log.info(f"  Result flags:  0b{flags:05b}")
        log.info(f"  Detail byte:   0x{detail:02X}")

        if detail == 0xAD:
            log.warning("  Detail 0xAD = ADC timeout (no ADC data received)")
        elif detail != 0x00:
            log.info(f"  Detail indicates first BRAM fail at addr[3:0] = {detail & 0x0F}")

        return all_pass

    def _save_adc_dump(self):
        """Save captured ADC samples to numpy file."""
        if not self._adc_samples and self.conn._mock:
            # In mock mode, generate synthetic ADC data
            self._adc_samples = list(np.random.randint(0, 65536, 256, dtype=np.uint16))

        if self._adc_samples:
            arr = np.array(self._adc_samples, dtype=np.uint16)
            np.save(self.adc_dump_path, arr)
            log.info(f"ADC raw data saved: {self.adc_dump_path} ({len(arr)} samples)")
        else:
            log.warning("No ADC samples captured for dump")


def main():
    parser = argparse.ArgumentParser(description="AERIS-10 Board Smoke Test")
    parser.add_argument("--live", action="store_true",
                        help="Use real FT2232H hardware (default: mock)")
    parser.add_argument("--device", type=int, default=0,
                        help="FT2232H device index")
    parser.add_argument("--adc-dump", type=str, default=None,
                        help="Save raw ADC samples to .npy file")
    args = parser.parse_args()

    mock_mode = not args.live
    conn = FT2232HConnection(mock=mock_mode)

    tester = SmokeTest(conn, adc_dump_path=args.adc_dump)
    success = tester.run()

    if conn.is_open:
        conn.close()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
