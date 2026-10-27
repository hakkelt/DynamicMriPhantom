import os
import shutil
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "python_serial"))
sys.path.insert(0, os.path.join(ROOT, "emulator"))

import emulator as emumod  # noqa: E402
import phantom_core as pc  # noqa: E402


def board_caps(board="atmega328pb", **overrides):
    """A copy of the default CAPS of ``board`` with ``overrides`` applied."""
    return {**pc.DEFAULT_CAPS[board], **overrides}


@pytest.fixture(scope="session")
def hex_path():
    """The ATmega328P firmware hex for the emulator (built on demand)."""
    if not emumod.node_available():
        pytest.skip("node or emulator/avr/node_modules missing (run npm ci in emulator/avr)")
    if os.environ.get("PHANTOM_REBUILD") or not os.path.exists(emumod.DEFAULT_HEX):
        if shutil.which("avr-g++") is None and not os.environ.get("AVR_GCC_BIN"):
            pytest.skip("firmware hex missing and no avr-g++ to build it")
        return emumod.build_hex()
    return emumod.DEFAULT_HEX


@pytest.fixture(scope="session")
def esp32_bin():
    """The host ESP32-S3 emulator (built on demand with the host C++ compiler)."""
    if os.environ.get("PHANTOM_REBUILD") or not os.path.exists(emumod.ESP32_BIN):
        if shutil.which(os.environ.get("CXX", "g++")) is None:
            pytest.skip("no host C++ compiler to build the ESP32 emulator")
        return emumod.build_esp32()
    return emumod.ESP32_BIN
