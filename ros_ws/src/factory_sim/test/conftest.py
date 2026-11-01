"""pytest setup: make `factory_sim` (this package's source) and `factory_core` / `policies` (project root) importable."""
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
