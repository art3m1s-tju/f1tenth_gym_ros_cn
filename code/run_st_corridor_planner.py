#!/usr/bin/env python3
"""Entry point wrapper for the ST-corridor local planner."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnc_rc.corridor.st_corridor_planner import main

if __name__ == "__main__":
    main()
