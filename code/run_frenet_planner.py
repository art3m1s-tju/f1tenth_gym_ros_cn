#!/usr/bin/env python3
"""Entry point wrapper for the Frenet static obstacle planner."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnc_rc.frenet.node import main

if __name__ == "__main__":
    main()
