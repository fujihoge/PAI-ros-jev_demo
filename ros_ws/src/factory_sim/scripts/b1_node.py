#!/usr/bin/env python3
"""Entry point of the B1 replay node (the code is in the factory_sim Python package)."""
import sys

from factory_sim.b1_node import main

if __name__ == "__main__":
    sys.exit(main())
