"""
SBRC (SnoBot Remote Controller) v1.0.0

Bluetooth RFCOMM server exposing a JSON-line command interface
for an ESP32-based handheld controller.
"""

__version__ = "1.0.0"

from .api import ControllerAPI
from .server import BluetoothServer

__all__ = ["ControllerAPI", "BluetoothServer"]
