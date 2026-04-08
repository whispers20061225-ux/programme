#!/usr/bin/env python3
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).with_name("tactile_grasp_studio_probe.py")), run_name="__main__")
