#!/usr/bin/env bash
set -euo pipefail

if python3 -m venv .venv 2>/dev/null; then
  echo "Created .venv with python3 -m venv"
else
  echo "python3 -m venv failed (likely missing python3-venv / ensurepip)."
  echo "Install system package, then rerun:"
  echo "  sudo apt install python3.13-venv"
  exit 1
fi

. .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
echo "Virtual environment ready."
