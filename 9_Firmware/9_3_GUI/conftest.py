"""
Pytest configuration for 9_Firmware/9_3_GUI tests.

Adds the current directory to sys.path so that the `v7` package
can be imported by test_v7.py when pytest runs from the repo root.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
