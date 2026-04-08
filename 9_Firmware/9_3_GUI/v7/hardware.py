"""
v7.hardware — Hardware interface classes for the PLFM Radar GUI V7.

Provides two USB hardware interfaces:
  - FT2232HQInterface  (PRIMARY — USB 2.0, VID 0x0403 / PID 0x6010)
  - STM32USBInterface   (USB CDC for commands and GPS)
"""

import logging
import struct
from typing import Dict, List, Optional

from .models import (
    FTDI_AVAILABLE,
    USB_AVAILABLE,
    RadarSettings,
)

if USB_AVAILABLE:
    import usb.core
    import usb.util

if FTDI_AVAILABLE:
    from pyftdi.ftdi import Ftdi
    from pyftdi.usbtools import UsbTools

logger = logging.getLogger(__name__)


# =============================================================================
# FT2232HQ Interface — PRIMARY data path (USB 2.0)
# =============================================================================

class FT2232HQInterface:
    """
    Interface for FT2232HQ (USB 2.0 Hi-Speed) in synchronous FIFO mode.

    This is the **primary** radar data interface.
    VID/PID: 0x0403 / 0x6010
    """

    VID = 0x0403
    PID = 0x6010

    def __init__(self):
        self.ftdi: Optional[object] = None
        self.is_open: bool = False

    # ---- enumeration -------------------------------------------------------

    def list_devices(self) -> List[Dict]:
        """List available FT2232H devices using pyftdi."""
        if not FTDI_AVAILABLE:
            logger.warning("pyftdi not available — cannot enumerate FT2232H devices")
            return []

        try:
            devices = []
            for device_desc in UsbTools.find_all([(self.VID, self.PID)]):
                devices.append({
                    "description": f"FT2232H Device {device_desc}",
                    "url": f"ftdi://{device_desc}/1",
                })
            return devices
        except Exception as e:
            logger.error(f"Error listing FT2232H devices: {e}")
            return []

    # ---- open / close ------------------------------------------------------

    def open_device(self, device_url: str) -> bool:
        """Open FT2232H device in synchronous FIFO mode."""
        if not FTDI_AVAILABLE:
            logger.error("pyftdi not available — cannot open device")
            return False

        try:
            self.ftdi = Ftdi()
            self.ftdi.open_from_url(device_url)

            # Synchronous FIFO mode
            self.ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)

            # Low-latency timer (2 ms)
            self.ftdi.set_latency_timer(2)

            # Purge stale data
            self.ftdi.purge_buffers()

            self.is_open = True
            logger.info(f"FT2232H device opened: {device_url}")
            return True
        except Exception as e:
            logger.error(f"Error opening FT2232H device: {e}")
            self.ftdi = None
            return False

    def close(self):
        """Close FT2232H device."""
        if self.ftdi and self.is_open:
            try:
                self.ftdi.close()
            except Exception as e:
                logger.error(f"Error closing FT2232H device: {e}")
            finally:
                self.is_open = False
                self.ftdi = None

    # ---- data I/O ----------------------------------------------------------

    def read_data(self, bytes_to_read: int = 4096) -> Optional[bytes]:
        """Read data from FT2232H."""
        if not self.is_open or self.ftdi is None:
            return None

        try:
            data = self.ftdi.read_data(bytes_to_read)
            if data:
                return bytes(data)
            return None
        except Exception as e:
            logger.error(f"Error reading from FT2232H: {e}")
            return None


# =============================================================================
# STM32 USB CDC Interface — commands & GPS data
# =============================================================================

class STM32USBInterface:
    """
    Interface for STM32 USB CDC (Virtual COM Port).

    Used to:
      - Send start flag and radar settings to the MCU
      - Receive GPS data from the MCU
    """

    STM32_VID_PIDS = [
        (0x0483, 0x5740),   # STM32 Virtual COM Port
        (0x0483, 0x3748),   # STM32 Discovery
        (0x0483, 0x374B),
        (0x0483, 0x374D),
        (0x0483, 0x374E),
        (0x0483, 0x3752),
    ]

    def __init__(self):
        self.device = None
        self.is_open: bool = False
        self.ep_in = None
        self.ep_out = None

    # ---- enumeration -------------------------------------------------------

    def list_devices(self) -> List[Dict]:
        """List available STM32 USB CDC devices."""
        if not USB_AVAILABLE:
            logger.warning("pyusb not available — cannot enumerate STM32 devices")
            return []

        devices = []
        try:
            for vid, pid in self.STM32_VID_PIDS:
                found = usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
                for dev in found:
                    try:
                        product = (usb.util.get_string(dev, dev.iProduct)
                                   if dev.iProduct else "STM32 CDC")
                        serial = (usb.util.get_string(dev, dev.iSerialNumber)
                                  if dev.iSerialNumber else "Unknown")
                        devices.append({
                            "description": f"{product} ({serial})",
                            "vendor_id": vid,
                            "product_id": pid,
                            "device": dev,
                        })
                    except Exception:
                        devices.append({
                            "description": f"STM32 CDC (VID:{vid:04X}, PID:{pid:04X})",
                            "vendor_id": vid,
                            "product_id": pid,
                            "device": dev,
                        })
        except Exception as e:
            logger.error(f"Error listing STM32 devices: {e}")
        return devices

    # ---- open / close ------------------------------------------------------

    def open_device(self, device_info: Dict) -> bool:
        """Open STM32 USB CDC device."""
        if not USB_AVAILABLE:
            logger.error("pyusb not available — cannot open STM32 device")
            return False

        try:
            self.device = device_info["device"]

            if self.device.is_kernel_driver_active(0):
                self.device.detach_kernel_driver(0)

            self.device.set_configuration()
            cfg = self.device.get_active_configuration()
            intf = cfg[(0, 0)]

            self.ep_out = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: (
                    usb.util.endpoint_direction(e.bEndpointAddress)
                    == usb.util.ENDPOINT_OUT
                ),
            )
            self.ep_in = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: (
                    usb.util.endpoint_direction(e.bEndpointAddress)
                    == usb.util.ENDPOINT_IN
                ),
            )

            if self.ep_out is None or self.ep_in is None:
                logger.error("Could not find STM32 CDC endpoints")
                return False

            self.is_open = True
            logger.info(f"STM32 USB device opened: {device_info.get('description', '')}")
            return True
        except Exception as e:
            logger.error(f"Error opening STM32 device: {e}")
            return False

    def close(self):
        """Close STM32 USB device."""
        if self.device and self.is_open:
            try:
                usb.util.dispose_resources(self.device)
            except Exception as e:
                logger.error(f"Error closing STM32 device: {e}")
        self.is_open = False
        self.device = None
        self.ep_in = None
        self.ep_out = None

    # ---- commands ----------------------------------------------------------

    def send_start_flag(self) -> bool:
        """Send start flag to STM32 (4-byte magic)."""
        start_packet = bytes([23, 46, 158, 237])
        logger.info("Sending start flag to STM32 via USB...")
        return self._send_data(start_packet)

    def send_settings(self, settings: RadarSettings) -> bool:
        """Send radar settings binary packet to STM32."""
        try:
            packet = self._create_settings_packet(settings)
            logger.info("Sending radar settings to STM32 via USB...")
            return self._send_data(packet)
        except Exception as e:
            logger.error(f"Error sending settings via USB: {e}")
            return False

    # ---- data I/O ----------------------------------------------------------

    def read_data(self, size: int = 64, timeout: int = 1000) -> Optional[bytes]:
        """Read data from STM32 via USB CDC."""
        if not self.is_open or self.ep_in is None:
            return None
        try:
            data = self.ep_in.read(size, timeout=timeout)
            return bytes(data)
        except Exception:
            # Timeout or other USB error
            return None

    # ---- internal helpers --------------------------------------------------

    def _send_data(self, data: bytes) -> bool:
        if not self.is_open or self.ep_out is None:
            return False
        try:
            packet_size = 64
            for i in range(0, len(data), packet_size):
                chunk = data[i : i + packet_size]
                if len(chunk) < packet_size:
                    chunk += b"\x00" * (packet_size - len(chunk))
                self.ep_out.write(chunk)
            return True
        except Exception as e:
            logger.error(f"Error sending data via USB: {e}")
            return False

    @staticmethod
    def _create_settings_packet(settings: RadarSettings) -> bytes:
        """Create binary settings packet: 'SET' ... 'END'."""
        packet = b"SET"
        packet += struct.pack(">d", settings.system_frequency)
        packet += struct.pack(">d", settings.chirp_duration_1)
        packet += struct.pack(">d", settings.chirp_duration_2)
        packet += struct.pack(">I", settings.chirps_per_position)
        packet += struct.pack(">d", settings.freq_min)
        packet += struct.pack(">d", settings.freq_max)
        packet += struct.pack(">d", settings.prf1)
        packet += struct.pack(">d", settings.prf2)
        packet += struct.pack(">d", settings.max_distance)
        packet += struct.pack(">d", settings.map_size)
        packet += b"END"
        return packet
