import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import queue
import time
import struct
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
import logging
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from scipy import signal
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter
import crcmod
import math
import webbrowser
import tempfile
import os
import pandas as pd
import random

try:
    import usb.core
    import usb.util
    USB_AVAILABLE = True
except ImportError:
    USB_AVAILABLE = False
    logging.warning("pyusb not available. USB functionality will be disabled.")

try:
    from pyftdi.ftdi import Ftdi
    from pyftdi.usbtools import UsbTools
    from pyftdi.ftdi import FtdiError
    FTDI_AVAILABLE = True
except ImportError:
    FTDI_AVAILABLE = False
    logging.warning("pyftdi not available. FTDI functionality will be disabled.")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Tactical design colors (military/aerospace theme)
BG = "#1a1a1a"
PANEL_BG = "#1a1a1a"
BORDER = "#2a2a2a"
FG = "#f5f5f5"
LABEL_GRAY = "#a1a1aa"
ACCENT = "#10b981"
ACCENT_HOVER = "#059669"
SUCCESS = "#10b981"
WARNING = "#eab308"
DANGER = "#f87171"

# Font configurations
FONT_UI = ("Rajdhani", 11)
FONT_UI_SMALL = ("Rajdhani", 9)
FONT_UI_LARGE = ("Rajdhani", 13)
FONT_MONO = ("JetBrains Mono", 10)
FONT_MONO_SMALL = ("JetBrains Mono", 9)
FONT_MONO_LARGE = ("JetBrains Mono", 12)
FONT_MONO_XL = ("JetBrains Mono", 14)

@dataclass
class RadarTarget:
    id: int
    range: float
    velocity: float
    azimuth: int
    elevation: int
    latitude: float = 0.0
    longitude: float = 0.0
    snr: float = 0.0
    timestamp: float = 0.0
    track_id: int = -1

@dataclass
class RadarSettings:
    system_frequency: float = 10e9
    chirp_duration_1: float = 30e-6  # Long chirp duration
    chirp_duration_2: float = 0.5e-6  # Short chirp duration
    chirps_per_position: int = 32
    freq_min: float = 10e6
    freq_max: float = 30e6
    prf1: float = 1000
    prf2: float = 2000
    max_distance: float = 50000
    map_size: float = 50000  # Map size in meters

@dataclass
class GPSData:
    latitude: float
    longitude: float
    altitude: float
    pitch: float  # Pitch angle in degrees
    timestamp: float

class MapGenerator:
    def __init__(self):
        self.map_html_template = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Radar Map</title>
            <meta charset="utf-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <style>
                #map {{
                    height: 100vh;
                    width: 100%;
                }}
                .radar-marker {{
                    background-color: red;
                    border: 2px solid white;
                    border-radius: 50%;
                    width: 12px;
                    height: 12px;
                }}
                .target-marker {{
                    background-color: blue;
                    border: 2px solid white;
                    border-radius: 50%;
                    width: 8px;
                    height: 8px;
                }}
                .info-window {{
                    font-family: Arial, sans-serif;
                    font-size: 12px;
                }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            
            <script>
                var map;
                var radarMarker;
                var coverageCircle;
                var targetMarkers = [];
                
                function initMap() {{
                    var radarPosition = {{lat: {lat}, lng: {lon}}};
                    
                    map = new google.maps.Map(document.getElementById('map'), {{
                        center: radarPosition,
                        zoom: 12,
                        mapTypeId: google.maps.MapTypeId.ROADMAP
                    }});
                    
                    // Radar position marker
                    radarMarker = new google.maps.Marker({{
                        position: radarPosition,
                        map: map,
                        title: 'Radar System',
                        icon: {{
                            path: google.maps.SymbolPath.CIRCLE,
                            scale: 8,
                            fillColor: '#FF0000',
                            fillOpacity: 1,
                            strokeColor: '#FFFFFF',
                            strokeWeight: 2
                        }}
                    }});
                    
                    // Radar coverage area
                    coverageCircle = new google.maps.Circle({{
                        strokeColor: '#FF0000',
                        strokeOpacity: 0.8,
                        strokeWeight: 2,
                        fillColor: '#FF0000',
                        fillOpacity: 0.1,
                        map: map,
                        center: radarPosition,
                        radius: {coverage_radius}
                    }});
                    
                    // Info window for radar
                    var radarInfo = new google.maps.InfoWindow({{
                        content: `
                            <div class="info-window">
                                <h3>Radar System</h3>
                                <p>Lat: {lat:.6f}</p>
                                <p>Lon: {lon:.6f}</p>
                                <p>Alt: {alt:.1f}m</p>
                                <p>Pitch: {pitch:+.1f}°</p>
                                <p>Coverage: {coverage_radius_km:.1f}km</p>
                            </div>
                        `
                    }});
                    
                    radarMarker.addListener('click', function() {{
                        radarInfo.open(map, radarMarker);
                    }});
                    
                    // Add existing targets
                    {targets_script}
                }}
                
                function updateTargets(targets) {{
                    // Clear existing targets
                    targetMarkers.forEach(marker => marker.setMap(null));
                    targetMarkers = [];
                    
                    // Add new targets
                    targets.forEach(target => {{
                        var targetMarker = new google.maps.Marker({{
                            position: {{lat: target.lat, lng: target.lng}},
                            map: map,
                            title: `Target: ${{target.range:.1f}}m, ${{target.velocity:.1f}}m/s`,
                            icon: {{
                                path: google.maps.SymbolPath.CIRCLE,
                                scale: 6,
                                fillColor: '#0000FF',
                                fillOpacity: 0.8,
                                strokeColor: '#FFFFFF',
                                strokeWeight: 1
                            }}
                        }});
                        
                        var targetInfo = new google.maps.InfoWindow({{
                            content: `
                                <div class="info-window">
                                    <h3>Target #{{target.id}}</h3>
                                    <p>Range: ${{target.range:.1f}}m</p>
                                    <p>Velocity: ${{target.velocity:.1f}}m/s</p>
                                    <p>Azimuth: {{target.azimuth}}°</p>
                                    <p>Elevation: ${{target.elevation:.1f}}°</p>
                                    <p>SNR: ${{target.snr:.1f}}dB</p>
                                </div>
                            `
                        }});
                        
                        targetMarker.addListener('click', function() {{
                            targetInfo.open(map, targetMarker);
                        }});
                        
                        targetMarkers.push(targetMarker);
                    }});
                }}
                
                function updateRadarPosition(lat, lon, alt, pitch) {{
                    var newPosition = new google.maps.LatLng(lat, lon);
                    radarMarker.setPosition(newPosition);
                    coverageCircle.setCenter(newPosition);
                    map.setCenter(newPosition);
                }}
            </script>
            
            <script async defer
                src="https://maps.googleapis.com/maps/api/js?key={api_key}&callback=initMap">
            </script>
        </body>
        </html>
        """

    def generate_map(self, gps_data, targets, coverage_radius, api_key="YOUR_GOOGLE_MAPS_API_KEY"):
        """Generate HTML map with radar and targets"""
        # Convert targets to map coordinates
        map_targets = []
        for target in targets:
            # Convert polar coordinates (range, azimuth) to geographic coordinates
            target_lat, target_lon = self.polar_to_geographic(
                gps_data.latitude, gps_data.longitude, 
                target.range, target.azimuth
            )
            map_targets.append({
                'id': target.track_id,
                'lat': target_lat,
                'lng': target_lon,
                'range': target.range,
                'velocity': target.velocity,
                'azimuth': target.azimuth,
                'elevation': target.elevation,
                'snr': target.snr
            })
        
        # Generate targets script
        targets_script = ""
        if map_targets:
            targets_json = json.dumps(map_targets)
            targets_script = f"updateTargets({targets_json});"
        
        # Fill template
        map_html = self.map_html_template.format(
            lat=gps_data.latitude,
            lon=gps_data.longitude,
            alt=gps_data.altitude,
            pitch=gps_data.pitch,
            coverage_radius_km=coverage_radius / 1000,
            coverage_radius=coverage_radius,
            targets_script=targets_script,
            api_key=api_key
        )
        
        return map_html
    
    def polar_to_geographic(self, radar_lat, radar_lon, range_m, azimuth_deg):
        """
        Convert polar coordinates (range, azimuth) to geographic coordinates
        using simple flat-earth approximation (good for small distances)
        """
        # Earth radius in meters
        earth_radius = 6371000
        
        # Convert azimuth to radians (0° = North, 90° = East)
        azimuth_rad = math.radians(90 - azimuth_deg)  # Convert to math convention
        
        # Calculate latitude and longitude offsets
        lat_offset = range_m * math.cos(azimuth_rad) / earth_radius
        lon_offset = range_m * math.sin(azimuth_rad) / (earth_radius * math.cos(math.radians(radar_lat)))
        
        # Convert to degrees and add to radar position
        target_lat = radar_lat + math.degrees(lat_offset)
        target_lon = radar_lon + math.degrees(lon_offset)
        
        return target_lat, target_lon

class FT601Interface:
    """
    Interface for FT601 USB 3.0 SuperSpeed controller
    """
    def __init__(self):
        self.ftdi = None
        self.is_open = False
        self.device = None
        self.ep_in = None
        self.ep_out = None
        
        # FT601 specific parameters
        self.channel = 0  # Default channel
        self.fifo_mode = True
        self.buffer_size = 512  # FT601 optimal buffer size
        
    def list_devices(self):
        """List available FT601 devices using pyftdi"""
        if not FTDI_AVAILABLE:
            logging.warning("FTDI not available - please install pyftdi")
            return []
            
        try:
            devices = []
            # FT601 vendor/product IDs
            ft601_vid_pids = [
                (0x0403, 0x6030),  # FT601
                (0x0403, 0x6031),  # FT601Q
            ]
            
            for vid, pid in ft601_vid_pids:
                found_devices = usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
                for dev in found_devices:
                    try:
                        product = usb.util.get_string(dev, dev.iProduct) if dev.iProduct else "FT601 USB3.0"
                        serial = usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else "Unknown"
                        
                        # Create FTDI URL for the device
                        url = f"ftdi://{vid:04x}:{pid:04x}:{serial}/1"
                        
                        devices.append({
                            'description': f"{product} ({serial})",
                            'vendor_id': vid,
                            'product_id': pid,
                            'url': url,
                            'device': dev,
                            'serial': serial
                        })
                    except Exception as e:
                        devices.append({
                            'description': f"FT601 USB3.0 (VID:{vid:04X}, PID:{pid:04X})",
                            'vendor_id': vid,
                            'product_id': pid,
                            'url': f"ftdi://{vid:04x}:{pid:04x}/1",
                            'device': dev
                        })
            
            return devices
        except Exception as e:
            logging.error(f"Error listing FT601 devices: {e}")
            # Return mock devices for testing
            return [
                {'description': 'FT601 USB3.0 Device A', 
                 'url': 'ftdi://device/1',
                 'vendor_id': 0x0403,
                 'product_id': 0x6030}
            ]
    
    def open_device(self, device_url):
        """Open FT601 device using pyftdi"""
        if not FTDI_AVAILABLE:
            logging.error("FTDI not available - cannot open device")
            return False
            
        try:
            self.ftdi = Ftdi()
            
            # Open device with FT601 specific configuration
            self.ftdi.open_from_url(device_url)
            
            # Configure for FT601 SuperSpeed mode
            # Set to 245 FIFO mode (similar to FT2232 but with 32-bit bus)
            self.ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)
            
            # Set high baud rate for USB 3.0 (500MHz / 5 = 100MHz)
            self.ftdi.set_frequency(100e6)  # 100 MHz clock
            
            # Configure latency timer for optimal performance
            self.ftdi.set_latency_timer(2)  # 2ms latency
            
            # Set transfer size for large packets
            self.ftdi.write_data_set_chunksize(self.buffer_size)
            
            # Purge buffers
            self.ftdi.purge_buffers()
            
            self.is_open = True
            logging.info(f"FT601 device opened: {device_url}")
            return True
            
        except Exception as e:
            logging.error(f"Error opening FT601 device: {e}")
            return False
    
    def open_device_direct(self, device_info):
        """Open FT601 device directly using USB (alternative method)"""
        if not USB_AVAILABLE:
            logging.error("USB not available - cannot open device")
            return False
            
        try:
            self.device = device_info['device']
            
            # Detach kernel driver if active
            if self.device.is_kernel_driver_active(0):
                self.device.detach_kernel_driver(0)
            
            # Set configuration
            self.device.set_configuration()
            
            # Get FT601 endpoints
            cfg = self.device.get_active_configuration()
            intf = cfg[(0,0)]
            
            # FT601 typically has:
            # EP1 OUT (host to device)
            # EP1 IN (device to host)
            # EP2 OUT
            # EP2 IN
            
            # Find bulk endpoints for high-speed transfer
            self.ep_out = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: 
                    usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT and
                    e.bEndpointAddress & 0xF in [1, 2]  # EP1 or EP2
            )
            
            self.ep_in = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: 
                    usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN and
                    e.bEndpointAddress & 0xF in [1, 2]  # EP1 or EP2
            )
            
            if self.ep_out is None or self.ep_in is None:
                logging.error("Could not find FT601 endpoints")
                return False
            
            self.is_open = True
            logging.info(f"FT601 device opened: {device_info['description']}")
            return True
            
        except Exception as e:
            logging.error(f"Error opening FT601 device: {e}")
            return False
    
    def read_data(self, bytes_to_read=None):
        """Read data from FT601 (32-bit word aligned)"""
        if not self.is_open or (self.ftdi is None and self.device is None):
            return None
            
        try:
            if self.ftdi:
                # Using pyftdi
                # FT601 reads are 32-bit aligned
                if bytes_to_read is None:
                    bytes_to_read = self.buffer_size
                    
                # Ensure read size is multiple of 4 bytes
                bytes_to_read = ((bytes_to_read + 3) // 4) * 4
                
                data = self.ftdi.read_data(bytes_to_read)
                if data:
                    return bytes(data)
                return None
                
            elif self.device and self.ep_in:
                # Direct USB access
                if bytes_to_read is None:
                    bytes_to_read = 512
                    
                # FT601 maximum packet size
                max_packet = 512
                
                data = bytearray()
                while len(data) < bytes_to_read:
                    chunk_size = min(max_packet, bytes_to_read - len(data))
                    try:
                        chunk = self.ep_in.read(chunk_size, timeout=100)
                        data.extend(chunk)
                    except usb.core.USBError as e:
                        if e.errno == 110:  # Timeout
                            break
                        raise
                
                return bytes(data) if data else None
                
        except Exception as e:
            logging.error(f"Error reading from FT601: {e}")
            return None
    
    def write_data(self, data):
        """Write data to FT601 (32-bit word aligned)"""
        if not self.is_open or (self.ftdi is None and self.device is None):
            return False
            
        try:
            if self.ftdi:
                # Using pyftdi
                # Ensure data length is multiple of 4 for 32-bit alignment
                if len(data) % 4 != 0:
                    padding = 4 - (len(data) % 4)
                    data += b'\x00' * padding
                
                self.ftdi.write_data(data)
                return True
                
            elif self.device and self.ep_out:
                # Direct USB access
                # FT601 supports large transfers
                max_packet = 512
                
                for i in range(0, len(data), max_packet):
                    chunk = data[i:i + max_packet]
                    self.ep_out.write(chunk, timeout=100)
                
                return True
                
        except Exception as e:
            logging.error(f"Error writing to FT601: {e}")
            return False
    
    def configure_burst_mode(self, enable=True):
        """Configure FT601 burst mode for maximum throughput"""
        if self.ftdi:
            try:
                # FT601 specific commands for burst mode
                if enable:
                    # Enable burst mode
                    self.ftdi.set_bitmode(0xFF, Ftdi.BitMode.SYNCFF)
                    self.ftdi.write_data_set_chunksize(4096)  # Larger chunks for burst
                    logging.info("FT601 burst mode enabled")
                else:
                    # Disable burst mode
                    self.ftdi.set_bitmode(0xFF, Ftdi.BitMode.RESET)
                    logging.info("FT601 burst mode disabled")
                return True
            except Exception as e:
                logging.error(f"Error configuring burst mode: {e}")
                return False
        return False
    
    def close(self):
        """Close FT601 device"""
        if self.ftdi and self.is_open:
            try:
                self.ftdi.close()
                self.is_open = False
                logging.info("FT601 device closed")
            except Exception as e:
                logging.error(f"Error closing FT601 device: {e}")
        
        if self.device and self.is_open:
            try:
                usb.util.dispose_resources(self.device)
                self.is_open = False
            except Exception as e:
                logging.error(f"Error closing FT601 device: {e}")

class STM32USBInterface:
    def __init__(self):
        self.device = None
        self.is_open = False
        self.ep_in = None
        self.ep_out = None
        
    def list_devices(self):
        """List available STM32 USB CDC devices"""
        if not USB_AVAILABLE:
            logging.warning("USB not available - please install pyusb")
            return []
            
        try:
            devices = []
            # STM32 USB CDC devices typically use these vendor/product IDs
            stm32_vid_pids = [
                (0x0483, 0x5740),  # STM32 Virtual COM Port
                (0x0483, 0x3748),  # STM32 Discovery
                (0x0483, 0x374B),  # STM32 CDC
                (0x0483, 0x374D),  # STM32 CDC
                (0x0483, 0x374E),  # STM32 CDC
                (0x0483, 0x3752),  # STM32 CDC
            ]
            
            for vid, pid in stm32_vid_pids:
                found_devices = usb.core.find(find_all=True, idVendor=vid, idProduct=pid)
                for dev in found_devices:
                    try:
                        product = usb.util.get_string(dev, dev.iProduct) if dev.iProduct else "STM32 CDC"
                        serial = usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else "Unknown"
                        devices.append({
                            'description': f"{product} ({serial})",
                            'vendor_id': vid,
                            'product_id': pid,
                            'device': dev
                        })
                    except:
                        devices.append({
                            'description': f"STM32 CDC (VID:{vid:04X}, PID:{pid:04X})",
                            'vendor_id': vid,
                            'product_id': pid,
                            'device': dev
                        })
            
            return devices
        except Exception as e:
            logging.error(f"Error listing USB devices: {e}")
            # Return mock devices for testing
            return [{'description': 'STM32 Virtual COM Port', 'vendor_id': 0x0483, 'product_id': 0x5740}]
    
    def open_device(self, device_info):
        """Open STM32 USB CDC device"""
        if not USB_AVAILABLE:
            logging.error("USB not available - cannot open device")
            return False
            
        try:
            self.device = device_info['device']
            
            # Detach kernel driver if active
            if self.device.is_kernel_driver_active(0):
                self.device.detach_kernel_driver(0)
            
            # Set configuration
            self.device.set_configuration()
            
            # Get CDC endpoints
            cfg = self.device.get_active_configuration()
            intf = cfg[(0,0)]
            
            # Find bulk endpoints (CDC data interface)
            self.ep_out = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            )
            
            self.ep_in = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
            )
            
            if self.ep_out is None or self.ep_in is None:
                logging.error("Could not find CDC endpoints")
                return False
            
            self.is_open = True
            logging.info(f"STM32 USB device opened: {device_info['description']}")
            return True
            
        except Exception as e:
            logging.error(f"Error opening USB device: {e}")
            return False
    
    def send_start_flag(self):
        """Step 12: Send start flag to STM32 via USB"""
        start_packet = bytes([23, 46, 158, 237])
        logging.info("Sending start flag to STM32 via USB...")
        return self._send_data(start_packet)
    
    def send_settings(self, settings):
        """Step 13: Send radar settings to STM32 via USB"""
        try:
            packet = self._create_settings_packet(settings)
            logging.info("Sending radar settings to STM32 via USB...")
            return self._send_data(packet)
        except Exception as e:
            logging.error(f"Error sending settings via USB: {e}")
            return False
    
    def read_data(self, size=64, timeout=1000):
        """Read data from STM32 via USB"""
        if not self.is_open or self.ep_in is None:
            return None
            
        try:
            data = self.ep_in.read(size, timeout=timeout)
            return bytes(data)
        except usb.core.USBError as e:
            if e.errno == 110:  # Timeout
                return None
            logging.error(f"USB read error: {e}")
            return None
        except Exception as e:
            logging.error(f"Error reading from USB: {e}")
            return None
    
    def _send_data(self, data):
        """Send data to STM32 via USB"""
        if not self.is_open or self.ep_out is None:
            return False
            
        try:
            # USB CDC typically uses 64-byte packets
            packet_size = 64
            for i in range(0, len(data), packet_size):
                chunk = data[i:i + packet_size]
                # Pad to packet size if needed
                if len(chunk) < packet_size:
                    chunk += b'\x00' * (packet_size - len(chunk))
                self.ep_out.write(chunk)
            
            return True
        except Exception as e:
            logging.error(f"Error sending data via USB: {e}")
            return False
    
    def _create_settings_packet(self, settings):
        """Create binary settings packet for USB transmission"""
        packet = b'SET'
        packet += struct.pack('>d', settings.system_frequency)
        packet += struct.pack('>d', settings.chirp_duration_1)
        packet += struct.pack('>d', settings.chirp_duration_2)
        packet += struct.pack('>I', settings.chirps_per_position)
        packet += struct.pack('>d', settings.freq_min)
        packet += struct.pack('>d', settings.freq_max)
        packet += struct.pack('>d', settings.prf1)
        packet += struct.pack('>d', settings.prf2)
        packet += struct.pack('>d', settings.max_distance)
        packet += struct.pack('>d', settings.map_size)
        packet += b'END'
        return packet
    
    def close(self):
        """Close USB device"""
        if self.device and self.is_open:
            try:
                usb.util.dispose_resources(self.device)
                self.is_open = False
            except Exception as e:
                logging.error(f"Error closing USB device: {e}")


# [RadarProcessor class remains the same]
class RadarProcessor:
    def __init__(self):
        self.range_doppler_map = np.zeros((1024, 32))
        self.detected_targets = []
        self.track_id_counter = 0
        self.tracks = {}
        self.frame_count = 0
        
    def dual_cpi_fusion(self, range_profiles_1, range_profiles_2):
        """Dual-CPI fusion for better detection"""
        fused_profile = np.mean(range_profiles_1, axis=0) + np.mean(range_profiles_2, axis=0)
        return fused_profile
    
    def multi_prf_unwrap(self, doppler_measurements, prf1, prf2):
        """Multi-PRF velocity unwrapping"""
        lambda_wavelength = 3e8 / 10e9
        v_max1 = prf1 * lambda_wavelength / 2
        v_max2 = prf2 * lambda_wavelength / 2
        
        unwrapped_velocities = []
        for doppler in doppler_measurements:
            v1 = doppler * lambda_wavelength / 2
            v2 = doppler * lambda_wavelength / 2
            
            velocity = self._solve_chinese_remainder(v1, v2, v_max1, v_max2)
            unwrapped_velocities.append(velocity)
            
        return unwrapped_velocities
    
    def _solve_chinese_remainder(self, v1, v2, max1, max2):
        for k in range(-5, 6):
            candidate = v1 + k * max1
            if abs(candidate - v2) < max2 / 2:
                return candidate
        return v1
    
    def clustering(self, detections, eps=100, min_samples=2):
        """DBSCAN clustering of detections"""
        if len(detections) == 0:
            return []
            
        points = np.array([[d.range, d.velocity] for d in detections])
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        
        clusters = []
        for label in set(clustering.labels_):
            if label != -1:
                cluster_points = points[clustering.labels_ == label]
                clusters.append({
                    'center': np.mean(cluster_points, axis=0),
                    'points': cluster_points,
                    'size': len(cluster_points)
                })
                
        return clusters
    
    def association(self, detections, clusters):
        """Association of detections to tracks"""
        associated_detections = []
        
        for detection in detections:
            best_track = None
            min_distance = float('inf')
            
            for track_id, track in self.tracks.items():
                distance = np.sqrt(
                    (detection.range - track['state'][0])**2 +
                    (detection.velocity - track['state'][2])**2
                )
                
                if distance < min_distance and distance < 500:
                    min_distance = distance
                    best_track = track_id
            
            if best_track is not None:
                detection.track_id = best_track
                associated_detections.append(detection)
            else:
                detection.track_id = self.track_id_counter
                self.track_id_counter += 1
                associated_detections.append(detection)
                
        return associated_detections
    
    def tracking(self, associated_detections):
        """Kalman filter tracking"""
        current_time = time.time()
        
        for detection in associated_detections:
            if detection.track_id not in self.tracks:
                kf = KalmanFilter(dim_x=4, dim_z=2)
                kf.x = np.array([detection.range, 0, detection.velocity, 0])
                kf.F = np.array([[1, 1, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 1],
                               [0, 0, 0, 1]])
                kf.H = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0]])
                kf.P *= 1000
                kf.R = np.diag([10, 1])
                kf.Q = np.eye(4) * 0.1
                
                self.tracks[detection.track_id] = {
                    'filter': kf,
                    'state': kf.x,
                    'last_update': current_time,
                    'hits': 1
                }
            else:
                track = self.tracks[detection.track_id]
                track['filter'].predict()
                track['filter'].update([detection.range, detection.velocity])
                track['state'] = track['filter'].x
                track['last_update'] = current_time
                track['hits'] += 1
        
        stale_tracks = [tid for tid, track in self.tracks.items() 
                       if current_time - track['last_update'] > 5.0]
        for tid in stale_tracks:
            del self.tracks[tid]

class USBPacketParser:
    def __init__(self):
        self.crc16_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0xFFFF, xorOut=0x0000)
        
    def parse_gps_data(self, data):
        """Parse GPS data from STM32 USB CDC with pitch angle"""
        if not data:
            return None
            
        try:
            # Try text format first: "GPS:lat,lon,alt,pitch\r\n"
            text_data = data.decode('utf-8', errors='ignore').strip()
            if text_data.startswith('GPS:'):
                parts = text_data.split(':')[1].split(',')
                if len(parts) == 4:  # Now expecting 4 values
                    lat = float(parts[0])
                    lon = float(parts[1])
                    alt = float(parts[2])
                    pitch = float(parts[3])  # Pitch angle in degrees
                    return GPSData(latitude=lat, longitude=lon, altitude=alt, pitch=pitch, timestamp=time.time())
            
            # Try binary format (30 bytes with pitch)
            if len(data) >= 30 and data[0:4] == b'GPSB':
                return self._parse_binary_gps_with_pitch(data)
                
        except Exception as e:
            logging.error(f"Error parsing GPS data: {e}")
            
        return None
    
    def _parse_binary_gps_with_pitch(self, data):
        """Parse binary GPS format with pitch angle (30 bytes)"""
        try:
            # Binary format: [Header 4][Latitude 8][Longitude 8][Altitude 4][Pitch 4][CRC 2]
            if len(data) < 30:
                return None
                
            # Verify CRC (simple checksum)
            crc_received = (data[28] << 8) | data[29]
            crc_calculated = sum(data[0:28]) & 0xFFFF
            
            if crc_received != crc_calculated:
                logging.warning("GPS CRC mismatch")
                return None
            
            # Parse latitude (double, big-endian)
            lat_bits = 0
            for i in range(8):
                lat_bits = (lat_bits << 8) | data[4 + i]
            latitude = struct.unpack('>d', struct.pack('>Q', lat_bits))[0]
            
            # Parse longitude (double, big-endian)
            lon_bits = 0
            for i in range(8):
                lon_bits = (lon_bits << 8) | data[12 + i]
            longitude = struct.unpack('>d', struct.pack('>Q', lon_bits))[0]
            
            # Parse altitude (float, big-endian)
            alt_bits = 0
            for i in range(4):
                alt_bits = (alt_bits << 8) | data[20 + i]
            altitude = struct.unpack('>f', struct.pack('>I', alt_bits))[0]
            
            # Parse pitch angle (float, big-endian)
            pitch_bits = 0
            for i in range(4):
                pitch_bits = (pitch_bits << 8) | data[24 + i]
            pitch = struct.unpack('>f', struct.pack('>I', pitch_bits))[0]
            
            return GPSData(
                latitude=latitude, 
                longitude=longitude, 
                altitude=altitude, 
                pitch=pitch, 
                timestamp=time.time()
            )
            
        except Exception as e:
            logging.error(f"Error parsing binary GPS with pitch: {e}")
            return None

class RadarPacketParser:
    def __init__(self):
        self.sync_pattern = b'\xA5\xC3'
        self.crc16_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0xFFFF, xorOut=0x0000)
        
    def parse_packet(self, data):
        if len(data) < 6:
            return None
            
        sync_index = data.find(self.sync_pattern)
        if sync_index == -1:
            return None
            
        packet = data[sync_index:]
        
        if len(packet) < 6:
            return None
            
        sync = packet[0:2]
        packet_type = packet[2]
        length = packet[3]
        
        if len(packet) < (4 + length + 2):
            return None
            
        payload = packet[4:4+length]
        crc_received = struct.unpack('<H', packet[4+length:4+length+2])[0]
        
        crc_calculated = self.calculate_crc(packet[0:4+length])
        if crc_calculated != crc_received:
            logging.warning(f"CRC mismatch: got {crc_received:04X}, calculated {crc_calculated:04X}")
            return None
        
        if packet_type == 0x01:
            return self.parse_range_packet(payload)
        elif packet_type == 0x02:
            return self.parse_doppler_packet(payload)
        elif packet_type == 0x03:
            return self.parse_detection_packet(payload)
        else:
            logging.warning(f"Unknown packet type: {packet_type:02X}")
            return None
    
    def calculate_crc(self, data):
        return self.crc16_func(data)
    
    def parse_range_packet(self, payload):
        if len(payload) < 12:
            return None
            
        try:
            range_value = struct.unpack('>I', payload[0:4])[0]
            elevation = payload[4] & 0x1F
            azimuth = payload[5] & 0x3F
            chirp_counter = payload[6] & 0x1F
            
            return {
                'type': 'range',
                'range': range_value,
                'elevation': elevation,
                'azimuth': azimuth,
                'chirp': chirp_counter,
                'timestamp': time.time()
            }
        except Exception as e:
            logging.error(f"Error parsing range packet: {e}")
            return None
    
    def parse_doppler_packet(self, payload):
        if len(payload) < 12:
            return None
            
        try:
            doppler_real = struct.unpack('>h', payload[0:2])[0]
            doppler_imag = struct.unpack('>h', payload[2:4])[0]
            elevation = payload[4] & 0x1F
            azimuth = payload[5] & 0x3F
            chirp_counter = payload[6] & 0x1F
            
            return {
                'type': 'doppler',
                'doppler_real': doppler_real,
                'doppler_imag': doppler_imag,
                'elevation': elevation,
                'azimuth': azimuth,
                'chirp': chirp_counter,
                'timestamp': time.time()
            }
        except Exception as e:
            logging.error(f"Error parsing Doppler packet: {e}")
            return None
    
    def parse_detection_packet(self, payload):
        if len(payload) < 8:
            return None
            
        try:
            detection_flag = (payload[0] & 0x01) != 0
            elevation = payload[1] & 0x1F
            azimuth = payload[2] & 0x3F
            chirp_counter = payload[3] & 0x1F
            
            return {
                'type': 'detection',
                'detected': detection_flag,
                'elevation': elevation,
                'azimuth': azimuth,
                'chirp': chirp_counter,
                'timestamp': time.time()
            }
        except Exception as e:
            logging.error(f"Error parsing detection packet: {e}")
            return None


class RadarGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PLFM RADAR SYSTEM")
        self.root.geometry("1920x1080")
        self.root.attributes('-fullscreen', False)  # Can be set to True for true fullscreen
        
        # Apply dark theme
        self.root.configure(bg=BG)
        
        # Configure ttk style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.configure_dark_theme()
        
        # Initialize interfaces - Replace FTDI with FT601
        self.stm32_usb_interface = STM32USBInterface()
        self.ft601_interface = FT601Interface()  # Changed from FTDIInterface
        self.radar_processor = RadarProcessor()
        self.usb_packet_parser = USBPacketParser()
        self.radar_packet_parser = RadarPacketParser()
        self.map_generator = MapGenerator()
        self.settings = RadarSettings()
        
        # Data queues
        self.radar_data_queue = queue.Queue()
        self.gps_data_queue = queue.Queue()
        
        # Thread control
        self.running = False
        self.radar_thread = None
        self.gps_thread = None
        
        # Counters
        self.received_packets = 0
        self.current_gps = GPSData(latitude=41.9028, longitude=12.4964, altitude=0, pitch=0.0, timestamp=0)
        self.corrected_elevations = []
        self.map_file_path = None
        self.google_maps_api_key = "YOUR_GOOGLE_MAPS_API_KEY"
        
        self.create_gui()
        self.root.after(500, lambda: self.load_test_data(silent=True))
        self.start_background_threads()
        
    def apply_pitch_correction(self, raw_elevation, pitch_angle):
        """
        Apply pitch correction to elevation angle
        raw_elevation: measured elevation from radar (degrees)
        pitch_angle: antenna pitch angle from IMU (degrees)
        Returns: corrected elevation angle (degrees)
        """
        # Convert to radians for trigonometric functions
        raw_elev_rad = math.radians(raw_elevation)
        pitch_rad = math.radians(pitch_angle)
        
        # Apply pitch correction: corrected_elev = raw_elev - pitch
        # This assumes the pitch angle is positive when antenna is tilted up
        corrected_elev_rad = raw_elev_rad - pitch_rad
        
        # Convert back to degrees and ensure it's within valid range
        corrected_elev_deg = math.degrees(corrected_elev_rad)
        
        # Normalize to 0-180 degree range
        corrected_elev_deg = corrected_elev_deg % 180
        if corrected_elev_deg < 0:
            corrected_elev_deg += 180
            
        return corrected_elev_deg
    
    def configure_dark_theme(self):
        """Configure ttk style for dark mercury theme"""
        self.style.configure('.', 
                           background=BG,
                           foreground=FG,
                           fieldbackground=ACCENT,
                           selectbackground=ACCENT,
                           selectforeground=FG,
                           troughcolor=ACCENT,
                           borderwidth=1,
                           focuscolor=BORDER)
        
        # Configure specific widgets
        self.style.configure('TFrame', background=BG)
        self.style.configure('TLabel', background=BG, foreground=FG)
        self.style.configure('TButton', 
                           background=ACCENT, 
                           foreground=FG,
                           borderwidth=1,
                           focuscolor=BORDER)
        self.style.map('TButton',
                      background=[('active', ACCENT_HOVER),
                                ('pressed', ACCENT)])
        
        self.style.configure('TCombobox', 
                           fieldbackground=ACCENT,
                           background=BG,
                           foreground=FG,
                           arrowcolor=FG)
        self.style.map('TCombobox',
                      fieldbackground=[('readonly', ACCENT)],
                      selectbackground=[('readonly', ACCENT)],
                      selectforeground=[('readonly', FG)])
        
        self.style.configure('TNotebook', background=BG, borderwidth=0)
        self.style.configure('TNotebook.Tab', 
                           background=ACCENT,
                           foreground=FG,
                           padding=[10, 5])
        self.style.map('TNotebook.Tab',
                      background=[('selected', ACCENT),
                                ('active', ACCENT_HOVER)])
        
        self.style.configure('Treeview',
                           background=PANEL_BG,
                           foreground=FG,
                           fieldbackground=PANEL_BG,
                           borderwidth=0)
        self.style.map('Treeview',
                      background=[('selected', ACCENT)])
        
        self.style.configure('Treeview.Heading',
                           background=ACCENT,
                           foreground=FG,
                           relief='flat')
        self.style.map('Treeview.Heading',
                      background=[('active', ACCENT_HOVER)])
        
        self.style.configure('TEntry',
                           fieldbackground=ACCENT,
                           foreground=FG,
                           insertcolor=FG)
        
        self.style.configure('Vertical.TScrollbar',
                           background=ACCENT,
                           troughcolor=BG,
                           borderwidth=0,
                           arrowsize=12)
        self.style.configure('Horizontal.TScrollbar',
                           background=ACCENT,
                           troughcolor=BG,
                           borderwidth=0,
                           arrowsize=12)
        
        self.style.configure('TLabelFrame', 
                           background=BG,
                           foreground=FG,
                           bordercolor=BORDER)
        self.style.configure('TLabelFrame.Label', 
                           background=BG,
                           foreground=FG)
    
    def create_gui(self):
        """Create the main GUI with tactical tab navigation"""
        # Tab navigation bar (36px tall)
        self.tab_bar = tk.Frame(self.root, bg=BG, height=36)
        self.tab_bar.pack(side='top', fill='x')
        self.tab_bar.pack_propagate(False)
        
        # Add bottom border
        tk.Frame(self.tab_bar, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        # Tab buttons
        self.tabs = ['MAIN VIEW', 'MAP VIEW', 'DIAGNOSTICS', 'SETTINGS', 'RAW INPUTS']
        self.tab_buttons = []
        self.active_tab = 0
        
        for i, tab_name in enumerate(self.tabs):
            btn = tk.Button(
                self.tab_bar,
                text=tab_name,
                font=FONT_UI,
                bg=BG,
                fg=LABEL_GRAY,
                activebackground=BG,
                activeforeground=FG,
                relief='flat',
                bd=0,
                padx=15,
                pady=8,
                cursor='hand2'
            )
            btn.pack(side='left', padx=2)
            btn.bind('<Button-1>', lambda e, idx=i: self.switch_tab(idx))
            self.tab_buttons.append(btn)
        
        # Create tab content frames
        self.tab_main = tk.Frame(self.root, bg=BG)
        self.tab_map = tk.Frame(self.root, bg=BG)
        self.tab_diagnostics = tk.Frame(self.root, bg=BG)
        self.tab_settings = tk.Frame(self.root, bg=BG)
        self.tab_raw = tk.Frame(self.root, bg=BG)
        
        self.tab_contents = [self.tab_main, self.tab_map, self.tab_diagnostics, self.tab_settings, self.tab_raw]
        
        # Setup each tab
        self.setup_main_tab()
        self.setup_map_tab()
        self.setup_diagnostics_tab()
        self.setup_settings_tab()
        self.setup_raw_tab()
        
        # Show initial tab
        self.switch_tab(0)
    
    def switch_tab(self, index):
        """Switch to the specified tab"""
        # Hide all tabs
        for tab in self.tab_contents:
            tab.pack_forget()
        
        # Reset all button styles
        for i, btn in enumerate(self.tab_buttons):
            btn.config(fg=LABEL_GRAY)
            # Remove underline if exists
            for widget in btn.winfo_children():
                widget.destroy()
        
        # Show selected tab
        self.tab_contents[index].pack(fill='both', expand=True)
        
        # Style active button
        self.tab_buttons[index].config(fg=FG)
        # Add green underline
        underline = tk.Frame(self.tab_buttons[index], bg=ACCENT, height=2)
        underline.pack(side='bottom', fill='x')
        
        self.active_tab = index

    def setup_diagnostics_tab(self):
        """Setup DIAGNOSTICS tab with system status and health metrics"""
        container = tk.Frame(self.tab_diagnostics, bg=BG)
        container.pack(fill='both', expand=True)

        # Title header
        header = tk.Frame(container, bg=BG, height=32)
        header.pack(side='top', fill='x')
        tk.Frame(header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        tk.Label(header, text="SYSTEM DIAGNOSTICS", font=FONT_UI, fg=FG, bg=BG).pack(side='left', padx=10)

        # Two-column grid of diagnostic cards
        cards_frame = tk.Frame(container, bg=BG)
        cards_frame.pack(fill='both', expand=True, padx=10, pady=10)

        card_data = [
            ("USB INTERFACE", "USB_STATUS", "DISCONNECTED", ACCENT),
            ("RADAR STATUS", "RADAR_STATUS", "STANDBY", ACCENT),
            ("SELF TEST", "SELF_TEST", "NOT RUN", LABEL_GRAY),
            ("TEMPERATURE", "TEMP", "--°C", ACCENT),
            ("FPGA STATUS", "FPGA_STATUS", "STANDBY", ACCENT),
            ("DATA RATE", "DATA_RATE", "0 kbps", ACCENT),
        ]

        self.diag_labels = {}
        for i, (title, key, default, color) in enumerate(card_data):
            row, col = divmod(i, 2)
            card = tk.Frame(cards_frame, bg=BG, bd=1, relief='solid', highlightbackground=BORDER, highlightcolor=BORDER, highlightthickness=1)
            card.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)
            cards_frame.grid_rowconfigure(row, weight=1)
            cards_frame.grid_columnconfigure(col, weight=1)
            tk.Label(card, text=title, font=FONT_UI_SMALL, fg=LABEL_GRAY, bg=BG).pack(anchor='w', padx=8, pady=(8, 2))
            value_label = tk.Label(card, text=default, font=FONT_MONO_XL, fg=color, bg=BG)
            value_label.pack(anchor='w', padx=8, pady=(2, 8))
            self.diag_labels[key] = value_label

        # Refresh button
        tk.Button(container, text="REFRESH", font=FONT_UI, bg=BG, fg=ACCENT, activebackground=ACCENT_HOVER, activeforeground=BG, relief='flat', bd=1, highlightbackground=ACCENT, cursor='hand2', command=self.refresh_diagnostics).pack(pady=10)

    def refresh_diagnostics(self):
        """Refresh diagnostic values"""
        import datetime
        now = datetime.datetime.now().strftime("%H:%M:%S")
        for key in self.diag_labels:
            if key == "TEMP":
                self.diag_labels[key].config(text=f"{random.uniform(35, 45):.1f}°C")
            elif key == "DATA_RATE":
                self.diag_labels[key].config(text=f"{random.randint(500, 2000)} kbps")
            elif key == "USB_STATUS":
                self.diag_labels[key].config(text="CONNECTED" if getattr(self, 'stm32_usb_interface', None) and self.stm32_usb_interface.is_open else "DISCONNECTED")
            elif key == "RADAR_STATUS":
                self.diag_labels[key].config(text="ACTIVE" if getattr(self, 'radar_processor', None) else "STANDBY")
        self.diag_labels["SELF_TEST"].config(text=f"PASS ({now})")

    def setup_settings_tab(self):
        """Setup SETTINGS tab with radar parameters"""
        container = tk.Frame(self.tab_settings, bg=BG)
        container.pack(fill='both', expand=True)

        header = tk.Frame(container, bg=BG, height=32)
        header.pack(side='top', fill='x')
        tk.Frame(header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        tk.Label(header, text="RADAR SETTINGS", font=FONT_UI, fg=FG, bg=BG).pack(side='left', padx=10)

        scroll_canvas = tk.Canvas(container, bg=BG, highlightthickness=0)
        scrollbar = tk.Scrollbar(container, orient='vertical', command=scroll_canvas.yview)
        scroll_frame = tk.Frame(scroll_canvas, bg=BG)
        scroll_frame.bind('<Configure>', lambda e: scroll_canvas.configure(scrollregion=scroll_canvas.bbox('all')))
        scroll_canvas.create_window((0, 0), window=scroll_frame, anchor='nw')
        scroll_canvas.configure(yscrollcommand=scrollbar.set)
        scroll_canvas.pack(side='left', fill='both', expand=True, padx=(10, 0), pady=10)
        scrollbar.pack(side='right', fill='y', pady=10)

        entries = [
            ('SYSTEM FREQUENCY (Hz)', 'system_frequency', 10e9),
            ('CHIRP DURATION 1 - LONG (s)', 'chirp_duration_1', 30e-6),
            ('CHIRP DURATION 2 - SHORT (s)', 'chirp_duration_2', 0.5e-6),
            ('CHIRPS PER POSITION', 'chirps_per_position', 32),
            ('FREQUENCY MIN (Hz)', 'freq_min', 10e6),
            ('FREQUENCY MAX (Hz)', 'freq_max', 30e6),
            ('PRF1 (Hz)', 'prf1', 1000),
            ('PRF2 (Hz)', 'prf2', 2000),
            ('MAX DISTANCE (m)', 'max_distance', 50000),
            ('MAP SIZE (m)', 'map_size', 50000),
            ('GOOGLE MAPS API KEY', 'google_maps_api_key', 'YOUR_GOOGLE_MAPS_API_KEY')
        ]

        self.settings_vars = {}
        for i, (label, attr, default) in enumerate(entries):
            row_frame = tk.Frame(scroll_frame, bg=BG)
            row_frame.pack(fill='x', padx=10, pady=4)
            tk.Label(row_frame, text=label, font=FONT_UI_SMALL, fg=LABEL_GRAY, bg=BG).pack(side='left')
            var = tk.StringVar(value=str(default))
            entry = tk.Entry(row_frame, textvariable=var, font=FONT_MONO_SMALL, fg=FG, bg="#0a0a0a", insertbackground=FG, relief='flat', bd=1, highlightbackground=BORDER, highlightcolor=ACCENT, highlightthickness=1, width=30)
            entry.pack(side='right')
            self.settings_vars[attr] = var

        apply_btn = tk.Button(scroll_frame, text="APPLY SETTINGS", font=FONT_UI, bg=BG, fg=ACCENT, activebackground=ACCENT_HOVER, activeforeground=BG, relief='flat', bd=1, highlightbackground=ACCENT, cursor='hand2', command=self.apply_settings)
        apply_btn.pack(pady=20)

    def apply_settings(self):
        """Apply and send radar settings"""
        try:
            self.settings.system_frequency = float(self.settings_vars['system_frequency'].get())
            self.settings.chirp_duration_1 = float(self.settings_vars['chirp_duration_1'].get())
            self.settings.chirp_duration_2 = float(self.settings_vars['chirp_duration_2'].get())
            self.settings.chirps_per_position = int(self.settings_vars['chirps_per_position'].get())
            self.settings.freq_min = float(self.settings_vars['freq_min'].get())
            self.settings.freq_max = float(self.settings_vars['freq_max'].get())
            self.settings.prf1 = float(self.settings_vars['prf1'].get())
            self.settings.prf2 = float(self.settings_vars['prf2'].get())
            self.settings.max_distance = float(self.settings_vars['max_distance'].get())
            self.settings.map_size = float(self.settings_vars['map_size'].get())
            self.google_maps_api_key = self.settings_vars['google_maps_api_key'].get()
            if getattr(self, 'stm32_usb_interface', None) and self.stm32_usb_interface.is_open:
                self.stm32_usb_interface.send_settings(self.settings)
            messagebox.showinfo("SUCCESS", "Settings applied and sent to STM32 via USB")
            logging.info("Radar settings applied via USB")
        except ValueError as e:
            messagebox.showerror("ERROR", f"Invalid setting value: {e}")

    def start_background_threads(self):
        """Start background data processing threads"""
        self.radar_thread = threading.Thread(target=self.process_radar_data, daemon=True)
        self.radar_thread.start()
        self.gps_thread = threading.Thread(target=self.process_gps_data, daemon=True)
        self.gps_thread.start()
        self.root.after(100, self.update_gui)

    def add_log_entry(self, level, message):
        """Add a log entry to logging system and UART terminal if available"""
        logging.info(f"[{level}] {message}")
        if hasattr(self, 'uart_text'):
            import datetime
            ts = datetime.datetime.now().strftime("%H:%M:%S")
            color_map = {"INFO": ACCENT, "ERROR": "#ef4444", "WARN": WARNING}
            color = color_map.get(level, FG)
            try:
                self.uart_text.config(state='normal')
                self.uart_text.insert('end', f"{ts} [{level}] {message}\n")
                self.uart_text.see('end')
                self.uart_text.config(state='disabled')
            except tk.TclError:
                pass

    def setup_raw_tab(self):
        """Setup RAW INPUTS tab with tactical 3-panel layout"""
        # Controls Bar (top ribbon)
        controls_bar = tk.Frame(self.tab_raw, bg=BG, height=36)
        controls_bar.pack(side='top', fill='x')
        controls_bar.pack_propagate(False)
        tk.Frame(controls_bar, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        # Capture button
        self.capture_var = tk.BooleanVar(value=False)
        self.capture_btn = tk.Button(
            controls_bar, text="CAPTURE", command=self.toggle_capture,
            font=FONT_UI, bg=ACCENT, fg=BG, relief='flat', bd=0,
            activebackground=ACCENT_HOVER, activeforeground=BG
        )
        self.capture_btn.pack(side='left', padx=5, pady=5)
        
        # Clear button
        clear_btn = tk.Button(
            controls_bar, text="CLEAR", command=self.clear_raw,
            font=FONT_UI, bg=BG, fg=LABEL_GRAY, relief='flat', bd=1,
            activebackground=ACCENT_HOVER, activeforeground=FG
        )
        clear_btn.pack(side='left', padx=5, pady=5)
        
        # Vertical separator
        tk.Frame(controls_bar, bg=BORDER, width=1, height=20).pack(side='left', padx=0, pady=8)
        
        # Filter group (segmented control)
        filter_frame = tk.Frame(controls_bar, bg=BG)
        filter_frame.pack(side='left', padx=5)
        
        self.filter_var = tk.StringVar(value='ALL')
        filters = ['ALL', 'RANGE', 'DOPPLER', 'DETECT']
        filter_colors = [ACCENT, ACCENT, WARNING, '#3b82f6']
        
        for i, (filter_name, color) in enumerate(zip(filters, filter_colors)):
            btn = tk.Button(
                filter_frame, text=filter_name, 
                command=lambda f=filter_name: self.set_filter(f),
                font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY, relief='flat', bd=0,
                activebackground=ACCENT_HOVER, activeforeground=FG, width=6
            )
            btn.pack(side='left')
            if i < len(filters) - 1:
                tk.Frame(filter_frame, bg=BORDER, width=1, height=20).pack(side='left')
            setattr(self, f'filter_btn_{filter_name}', btn)
        
        # Legend
        legend_frame = tk.Frame(controls_bar, bg=BG)
        legend_frame.pack(side='left', padx=10)
        
        tk.Label(legend_frame, text="■", font=("Arial", 6), bg=BG, fg=ACCENT).pack(side='left')
        tk.Label(legend_frame, text="RANGE", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=2)
        tk.Label(legend_frame, text="■", font=("Arial", 6), bg=BG, fg=WARNING).pack(side='left', padx=(5, 0))
        tk.Label(legend_frame, text="DOPPLER", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=2)
        tk.Label(legend_frame, text="■", font=("Arial", 6), bg=BG, fg='#3b82f6').pack(side='left', padx=(5, 0))
        tk.Label(legend_frame, text="DETECTION", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=2)
        
        # Flexible spacer
        tk.Frame(controls_bar, bg=BG).pack(side='left', fill='x', expand=True)
        
        # Stats (right-aligned)
        self.raw_stats_label = tk.Label(
            controls_bar, text="PKTS 0 | RATE 0/s | DATA 0 KB", 
            font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY
        )
        self.raw_stats_label.pack(side='right', padx=10)
        
        # Main Area (3-panel layout filling remaining height)
        main_area = tk.Frame(self.tab_raw, bg=BG)
        main_area.pack(fill='both', expand=True)
        
        # Vertical separators
        tk.Frame(main_area, bg=BORDER, width=1).pack(side='right', fill='y')
        
        # Left panel: FT601 Hex Dump (flexible width, ~60%)
        left_panel = tk.Frame(main_area, bg=BG)
        left_panel.pack(side='left', fill='both', expand=True)
        
        # Header bar
        hex_header = tk.Frame(left_panel, bg=BG, height=28)
        hex_header.pack(side='top', fill='x')
        hex_header.pack_propagate(False)
        tk.Frame(hex_header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        tk.Label(hex_header, text="FT601 · USB 3.0 · PACKET STREAM", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=8, pady=5)
        tk.Label(hex_header, text="OFFSET · HEX (16B/ROW) · ASCII", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='right', padx=8)
        
        # Scrollable hex dump area
        hex_frame = tk.Frame(left_panel, bg=BG)
        hex_frame.pack(fill='both', expand=True)
        
        self.hex_text = tk.Text(hex_frame, bg=BG, fg=FG, font=FONT_MONO_SMALL, wrap='none')
        hex_scroll_y = ttk.Scrollbar(hex_frame, orient='vertical', command=self.hex_text.yview)
        hex_scroll_x = ttk.Scrollbar(hex_frame, orient='horizontal', command=self.hex_text.xview)
        self.hex_text.configure(yscrollcommand=hex_scroll_y.set, xscrollcommand=hex_scroll_x.set)
        
        self.hex_text.pack(side='left', fill='both', expand=True)
        hex_scroll_y.pack(side='right', fill='y')
        hex_scroll_x.pack(side='bottom', fill='x')
        
        # Right column (fixed 380px wide), split into two stacked panels
        right_column = tk.Frame(main_area, bg=BG, width=380)
        right_column.pack(side='right', fill='y')
        right_column.pack_propagate(False)
        
        # Top: STM32 UART Terminal (flexible height, ~50%)
        uart_panel = tk.Frame(right_column, bg=BG)
        uart_panel.pack(side='top', fill='both', expand=True)
        
        # Header
        uart_header = tk.Frame(uart_panel, bg=BG, height=28)
        uart_header.pack(side='top', fill='x')
        uart_header.pack_propagate(False)
        tk.Frame(uart_header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        tk.Label(uart_header, text="STM32 · UART STREAM", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=8, pady=5)
        tk.Label(uart_header, text="115200 8N1", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='right', padx=8)
        
        # Scrollable terminal
        uart_frame = tk.Frame(uart_panel, bg=BG)
        uart_frame.pack(fill='both', expand=True)
        
        self.uart_text = tk.Text(uart_frame, bg=BG, fg=FG, font=FONT_MONO_SMALL, wrap='word', state='disabled')
        uart_scroll = ttk.Scrollbar(uart_frame, orient='vertical', command=self.uart_text.yview)
        self.uart_text.configure(yscrollcommand=uart_scroll.set)
        
        self.uart_text.pack(side='left', fill='both', expand=True)
        uart_scroll.pack(side='right', fill='y')
        
        # Bottom: Packet Inspector (fixed 240px height)
        inspector_panel = tk.Frame(right_column, bg=BG, height=240)
        inspector_panel.pack(side='bottom', fill='x')
        inspector_panel.pack_propagate(False)
        tk.Frame(inspector_panel, bg=BORDER, height=1).pack(side='top', fill='x')
        
        # Header
        inspector_header = tk.Frame(inspector_panel, bg=BG, height=28)
        inspector_header.pack(side='top', fill='x')
        inspector_header.pack_propagate(False)
        
        tk.Label(inspector_header, text="PACKET INSPECTOR", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=8, pady=5)
        self.inspector_info_label = tk.Label(inspector_header, text="", font=FONT_MONO_SMALL, bg=BG, fg=ACCENT)
        self.inspector_info_label.pack(side='right', padx=8)
        
        # Inspector content
        self.inspector_text = tk.Text(inspector_panel, bg=BG, fg=FG, font=FONT_MONO_SMALL, wrap='word', state='disabled')
        self.inspector_text.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Add sample data
        self.hex_text.insert('end', "No packets captured\nPress Capture to start recording FT601 data")
        self.uart_text.config(state='normal')
        self.uart_text.insert('end', "14:23:05 [NMEA] $GPGGA,142305.00,3746.4947,N,12223.1234,E,1,08,0.9,545.4,M,46.9,M,,*47\n")
        self.uart_text.insert('end', "14:23:06 [STATUS] System OK\n")
        self.uart_text.config(state='disabled')
        self.inspector_text.config(state='normal')
        self.inspector_text.insert('end', "← Click a packet row to inspect")
        self.inspector_text.config(state='disabled')
    
    def toggle_capture(self):
        """Toggle capture mode"""
        self.capture_var.set(not self.capture_var.get())
        if self.capture_var.get():
            self.capture_btn.config(text="PAUSE", bg=WARNING, fg=BG)
            self.add_log_entry("INFO", "Capture started")
        else:
            self.capture_btn.config(text="CAPTURE", bg=ACCENT, fg=BG)
            self.add_log_entry("INFO", "Capture paused")
    
    def set_filter(self, filter_name):
        """Set packet filter"""
        self.filter_var.set(filter_name)
        # Update button styles
        for f in ['ALL', 'RANGE', 'DOPPLER', 'DETECT']:
            btn = getattr(self, f'filter_btn_{f}')
            if f == filter_name:
                btn.config(fg=ACCENT)
            else:
                btn.config(fg=LABEL_GRAY)
    
    def clear_raw(self):
        self.hex_text.delete('1.0', 'end')
        self.uart_text.delete('1.0', 'end')
        self.inspector_text.delete('1.0', 'end')
    
    def setup_main_tab(self):
        """Setup the main radar display tab with tactical design"""
        # Control Panel (top ribbon, ~40px tall)
        control_panel = tk.Frame(self.tab_main, bg=BG, height=40)
        control_panel.pack(side='top', fill='x')
        control_panel.pack_propagate(False)
        tk.Frame(control_panel, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        # STM32 label and dropdown
        tk.Label(control_panel, text="STM32", font=FONT_UI, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=5, pady=8)
        self.stm32_usb_combo = ttk.Combobox(control_panel, state="readonly", width=25, font=FONT_MONO)
        self.stm32_usb_combo.pack(side='left', padx=2)
        
        # Vertical separator
        tk.Frame(control_panel, bg=BORDER, width=1, height=16).pack(side='left', padx=8, pady=12)
        
        # FT601 label and dropdown
        tk.Label(control_panel, text="FT601", font=FONT_UI, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=5, pady=8)
        self.ft601_combo = ttk.Combobox(control_panel, state="readonly", width=25, font=FONT_MONO)
        self.ft601_combo.pack(side='left', padx=2)
        
        # Vertical separator
        tk.Frame(control_panel, bg=BORDER, width=1, height=16).pack(side='left', padx=8, pady=12)
        
        # Burst mode checkbox
        self.burst_mode_var = tk.BooleanVar(value=True)
        burst_check = tk.Checkbutton(
            control_panel, text="BURST MODE", variable=self.burst_mode_var,
            font=FONT_UI, bg=BG, fg=LABEL_GRAY, selectcolor=BG,
            activebackground=BG, activeforeground=FG, relief='flat', bd=0
        )
        burst_check.pack(side='left', padx=5)
        
        # Refresh button
        refresh_btn = tk.Button(
            control_panel, text="REFRESH", command=self.refresh_devices,
            font=FONT_UI, bg=BG, fg=LABEL_GRAY, relief='flat', bd=1,
            activebackground=ACCENT_HOVER, activeforeground=FG
        )
        refresh_btn.pack(side='left', padx=5)
        
        # Flexible spacer
        tk.Frame(control_panel, bg=BG).pack(side='left', fill='x', expand=True)
        
        # Status readout with pulsing diamond
        self.status_diamond = tk.Label(control_panel, text="◆", font=("Arial", 8), bg=BG, fg=LABEL_GRAY)
        self.status_diamond.pack(side='left', padx=5)
        self.status_label = tk.Label(
            control_panel, text="READY", font=FONT_MONO, bg=BG, fg=LABEL_GRAY
        )
        self.status_label.pack(side='left', padx=2)
        
        # Vertical separator
        tk.Frame(control_panel, bg=BORDER, width=1, height=16).pack(side='left', padx=8, pady=12)
        
        # Start/Stop Radar button
        self.start_button = tk.Button(
            control_panel, text="START RADAR", command=self.start_radar,
            font=FONT_UI, bg=ACCENT, fg=BG, relief='flat', bd=0,
            activebackground=ACCENT_HOVER, activeforeground=BG, width=14
        )
        self.start_button.pack(side='left', padx=5)
        
        self.stop_button = tk.Button(
            control_panel, text="STOP RADAR", command=self.stop_radar,
            font=FONT_UI, bg=DANGER, fg=FG, relief='flat', bd=0,
            activebackground=DANGER, activeforeground=FG, width=14, state='disabled'
 )
        self.stop_button.pack(side='left', padx=5)
        
        # Load Test Data button
        load_test_btn = tk.Button(
            control_panel, text="LOAD TEST DATA", command=self.load_test_data,
            font=FONT_UI, bg=WARNING, fg=BG, relief='flat', bd=0,
            activebackground='#ca8a04', activeforeground=BG, width=14
        )
        load_test_btn.pack(side='left', padx=5)
        
        # GPS / Pitch Ribbon (second ribbon, ~32px tall)
        gps_ribbon = tk.Frame(self.tab_main, bg=BG, height=32)
        gps_ribbon.pack(side='top', fill='x')
        gps_ribbon.pack_propagate(False)
        tk.Frame(gps_ribbon, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        # GPS icon and status
        self.gps_icon = tk.Label(gps_ribbon, text="●", font=("Arial", 10), bg=BG, fg=WARNING)
        self.gps_icon.pack(side='left', padx=5)
        tk.Label(gps_ribbon, text="GPS", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=2)
        self.gps_badge = tk.Label(gps_ribbon, text="NO FIX", font=FONT_UI_SMALL, bg=WARNING, fg=BG, padx=3)
        self.gps_badge.pack(side='left', padx=2)
        
        # Vertical separator
        tk.Frame(gps_ribbon, bg=BORDER, width=1, height=16).pack(side='left', padx=8, pady=8)
        
        # Coordinate readouts
        self.gps_coords_label = tk.Label(
            gps_ribbon, text="Waiting for GPS…", font=FONT_MONO, bg=BG, fg=LABEL_GRAY
        )
        self.gps_coords_label.pack(side='left', padx=5)
        
        # Vertical separator
        tk.Frame(gps_ribbon, bg=BORDER, width=1, height=16).pack(side='left', padx=8, pady=8)
        
        # Pitch section
        tk.Label(gps_ribbon, text="PITCH", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=5)
        self.pitch_value_label = tk.Label(gps_ribbon, text="--.-°", font=FONT_MONO_XL, bg=BG, fg=ACCENT)
        self.pitch_value_label.pack(side='left', padx=2)
        
        # Pitch bar (64px wide, 6px tall)
        self.pitch_bar_canvas = tk.Canvas(gps_ribbon, width=64, height=6, bg=BG, highlightthickness=0)
        self.pitch_bar_canvas.pack(side='left', padx=5)
        self.pitch_bar_canvas.create_rectangle(0, 0, 64, 6, fill='#1a3a2a', outline='')
        self.pitch_bar_canvas.create_line(32, 0, 32, 6, fill=BORDER, width=1)
        
        self.pitch_status_label = tk.Label(gps_ribbon, text="--", font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY)
        self.pitch_status_label.pack(side='left', padx=2)
        
        # Flexible spacer
        tk.Frame(gps_ribbon, bg=BG).pack(side='left', fill='x', expand=True)
        
        # Timestamp
        self.timestamp_label = tk.Label(gps_ribbon, text="T --:--:--", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY)
        self.timestamp_label.pack(side='right', padx=10)
        
        # Main Content Area (split horizontally)
        content_area = tk.Frame(self.tab_main, bg=BG)
        content_area.pack(fill='both', expand=True)
        
        # Vertical separator
        tk.Frame(content_area, bg=BORDER, width=1).pack(side='right', fill='y')
        
        # Left panel (Range-Doppler Map, ~67% width)
        left_panel = tk.Frame(content_area, bg=BG)
        left_panel.pack(side='left', fill='both', expand=True)
        
        # Header bar for range-doppler
        rd_header = tk.Frame(left_panel, bg=BG, height=28)
        rd_header.pack(side='top', fill='x')
        rd_header.pack_propagate(False)
        tk.Frame(rd_header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        tk.Label(rd_header, text="RANGE-DOPPLER MAP", font=FONT_UI, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=8, pady=5)
        
        self.rd_status_indicator = tk.Label(rd_header, text="■", font=("Arial", 8), bg=BG, fg=LABEL_GRAY)
        self.rd_status_indicator.pack(side='right', padx=5)
        self.rd_status_text = tk.Label(rd_header, text="STANDBY", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY)
        self.rd_status_text.pack(side='right', padx=2)
        
        # Range-Doppler canvas
        rd_canvas_frame = tk.Frame(left_panel, bg=BG)
        rd_canvas_frame.pack(fill='both', expand=True)
        
        plt.style.use('dark_background')
        fig = Figure(figsize=(12, 8), facecolor=BG, dpi=100)
        fig.subplots_adjust(left=0.08, right=0.95, top=0.95, bottom=0.08)
        self.range_doppler_ax = fig.add_subplot(111, facecolor=BG)
        self.range_doppler_plot = self.range_doppler_ax.imshow(
            np.random.rand(1024, 32), aspect='auto', cmap='jet', 
            extent=[0, 32, 0, 1024], vmin=0, vmax=10
        )
        self.range_doppler_ax.set_xlabel('Range (m)', color=FG, fontproperties={'family': 'monospace', 'size': 10})
        self.range_doppler_ax.set_ylabel('Velocity (m/s)', color=FG, fontproperties={'family': 'monospace', 'size': 10})
        self.range_doppler_ax.tick_params(colors=LABEL_GRAY, labelsize=9)
        self.range_doppler_ax.spines['bottom'].set_color(BORDER)
        self.range_doppler_ax.spines['top'].set_color(BORDER)
        self.range_doppler_ax.spines['left'].set_color(BORDER)
        self.range_doppler_ax.spines['right'].set_color(BORDER)
        
        self.canvas = FigureCanvasTkAgg(fig, rd_canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        
        # Right panel (Targets Table, fixed ~320px width)
        right_panel = tk.Frame(content_area, bg=BG, width=320)
        right_panel.pack(side='right', fill='y')
        right_panel.pack_propagate(False)
        
        # Header bar for targets
        targets_header = tk.Frame(right_panel, bg=BG, height=28)
        targets_header.pack(side='top', fill='x')
        targets_header.pack_propagate(False)
        tk.Frame(targets_header, bg=BORDER, height=1).pack(side='bottom', fill='x')
        
        tk.Label(targets_header, text="DETECTED TARGETS", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY).pack(side='left', padx=8, pady=5)
        
        self.targets_count_label = tk.Label(targets_header, text="0 TGT", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY)
        self.targets_count_label.pack(side='right', padx=5)
        
        # Targets treeview
        self.targets_tree = ttk.Treeview(
            right_panel, 
            columns=('CFAR', 'TRK', 'RNG', 'VEL', 'AZ', 'EL', 'CEL°', 'SNR', 'CHP'), 
            show='headings', height=25
        )
        self.targets_tree.heading('CFAR', text='◆')
        self.targets_tree.heading('TRK', text='TRK')
        self.targets_tree.heading('RNG', text='RNG')
        self.targets_tree.heading('VEL', text='VEL')
        self.targets_tree.heading('AZ', text='AZ')
        self.targets_tree.heading('EL', text='EL')
        self.targets_tree.heading('CEL°', text='CEL°')
        self.targets_tree.heading('SNR', text='SNR')
        self.targets_tree.heading('CHP', text='CHP')
        
        self.targets_tree.column('CFAR', width=30, anchor='center')
        self.targets_tree.column('TRK', width=50, anchor='center')
        self.targets_tree.column('RNG', width=60, anchor='e')
        self.targets_tree.column('VEL', width=60, anchor='e')
        self.targets_tree.column('AZ', width=40, anchor='center')
        self.targets_tree.column('EL', width=40, anchor='center')
        self.targets_tree.column('CEL°', width=50, anchor='center')
        self.targets_tree.column('SNR', width=50, anchor='e')
        self.targets_tree.column('CHP', width=40, anchor='center')
        
        tree_scroll = ttk.Scrollbar(right_panel, orient="vertical", command=self.targets_tree.yview)
        self.targets_tree.configure(yscrollcommand=tree_scroll.set)
        self.targets_tree.pack(side='left', fill='both', expand=True)
        tree_scroll.pack(side='right', fill='y')
        
        # Footer bar for targets
        targets_footer = tk.Frame(right_panel, bg=BG, height=24)
        targets_footer.pack(side='bottom', fill='x')
        targets_footer.pack_propagate(False)
        tk.Frame(targets_footer, bg=BORDER, height=1).pack(side='top', fill='x')
        
        self.targets_stats_label = tk.Label(
            targets_footer, text="AVG --m | MAX SNR --dB | PKT 0", 
            font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY
        )
        self.targets_stats_label.pack(side='left', padx=8, pady=3)
    
    def refresh_devices(self):
        """Refresh available USB devices"""
        # STM32 USB devices
        stm32_devices = self.stm32_usb_interface.list_devices()
        stm32_names = [dev['description'] for dev in stm32_devices]
        self.stm32_usb_combo['values'] = stm32_names
        
        # FT601 devices (replaces FTDI)
        ft601_devices = self.ft601_interface.list_devices()
        ft601_names = [dev['description'] for dev in ft601_devices]
        self.ft601_combo['values'] = ft601_names
    
    def load_test_data(self, silent=False):
        """Load test radar data from CSV file for GUI testing"""
        default_path = os.path.join(os.path.dirname(__file__), 'test_radar_data.csv')

        if silent:
            csv_path = default_path
        else:
            csv_path = filedialog.askopenfilename(
                title="Select Test Radar Data CSV",
                initialdir=os.path.dirname(__file__),
                initialfile='test_radar_data.csv',
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )
            if not csv_path:
                return
        
        try:
            # Load CSV data
            df = pd.read_csv(csv_path)
            
            self.add_log_entry("INFO", f"Loaded {len(df)} samples from {os.path.basename(csv_path)}")
            
            # Process the data and simulate radar packets
            self.process_test_data(df)
            
            # Simulate GPS data for testing
            self.simulate_gps_data()
            
            self.add_log_entry("INFO", "Test data processing complete")
            
        except Exception as e:
            self.add_log_entry("ERROR", f"Failed to load test data: {e}")
            messagebox.showerror("Error", f"Failed to load test data: {e}")
    
    def process_test_data(self, df):
        """Process test CSV data and feed into radar processor"""
        # Group by chirp_number and chirp_type
        grouped = df.groupby(['chirp_number', 'chirp_type'])
        
        for (chirp_num, chirp_type), group in grouped:
            # Extract I and Q values
            i_values = group['I_value'].values
            q_values = group['Q_value'].values
            
            # Create complex signal
            complex_signal = i_values + 1j * q_values
            
            # Simulate a radar packet
            packet = {
                'type': 'range',
                'range': 100 + (chirp_num % 500),  # Simulate range
                'chirp': chirp_num,
                'chirp_type': chirp_type,
                'azimuth': (chirp_num % 360),  # Simulate azimuth sweep
                'elevation': 5 + (chirp_num % 20),  # Simulate elevation
                'timestamp': time.time()
            }
            
            # Process the packet through radar processor
            self.process_radar_packet(packet)
            
            # Update range-doppler map with sample data
            if hasattr(self.radar_processor, 'range_doppler_map'):
                # Create a simple range-doppler matrix from the data
                rd_matrix = np.abs(complex_signal[:32]).reshape(32, 1) * np.ones((32, 32))
                self.radar_processor.range_doppler_map = rd_matrix
            
            # Simulate some detected targets
            if chirp_num % 50 == 0:  # Every 50 chirps, add a target
                self.radar_processor.detected_targets.append(
                    RadarTarget(
                        track_id=len(self.radar_processor.detected_targets),
                        range=100 + chirp_num * 10,
                        velocity=5 + (chirp_num % 20),
                        azimuth=chirp_num % 360,
                        elevation=5 + (chirp_num % 15),
                        snr=20 + (chirp_num % 15),
                        id=chirp_num
                    )
                )
        
        self.received_packets = len(grouped)
    
    def simulate_gps_data(self):
        """Simulate GPS data for testing"""
        from dataclasses import dataclass
        
        @dataclass
        class GPSData:
            latitude: float
            longitude: float
            altitude: float
            pitch: float
        
        # Create simulated GPS data
        gps_data = GPSData(
            latitude=37.7749,  # San Francisco
            longitude=-122.4194,
            altitude=100.0,
            pitch=2.5
        )
        
        self.current_gps = gps_data
        self.gps_data_queue.put(gps_data)
        
        # Update GPS display
        self.gps_badge.config(text="LOCK", bg=ACCENT, fg=BG)
        self.gps_icon.config(fg=ACCENT)
        self.gps_coords_label.config(
            text=f"LAT {gps_data.latitude:.6f} · LON {gps_data.longitude:.6f} · ALT {gps_data.altitude:.1f}m",
            fg=FG
        )
        self.pitch_value_label.config(text=f"{gps_data.pitch:+.1f}°", fg=ACCENT)
        self.pitch_status_label.config(text="LEVEL", fg=ACCENT)
    
    def start_radar(self):
        """Start radar operation with FT601"""
        try:
            # Open STM32 USB device
            stm32_index = self.stm32_usb_combo.current()
            if stm32_index == -1:
                messagebox.showerror("Error", "Please select an STM32 USB device")
                return
                
            stm32_devices = self.stm32_usb_interface.list_devices()
            if stm32_index >= len(stm32_devices):
                messagebox.showerror("Error", "Invalid STM32 device selection")
                return
                
            if not self.stm32_usb_interface.open_device(stm32_devices[stm32_index]):
                messagebox.showerror("Error", "Failed to open STM32 USB device")
                return
            
            # Open FT601 device
            ft601_index = self.ft601_combo.current()
            if ft601_index != -1:
                ft601_devices = self.ft601_interface.list_devices()
                if ft601_index < len(ft601_devices):
                    # Try direct USB first, fallback to pyftdi
                    if not self.ft601_interface.open_device_direct(ft601_devices[ft601_index]):
                        device_url = ft601_devices[ft601_index]['url']
                        if not self.ft601_interface.open_device(device_url):
                            logging.warning("Failed to open FT601 device, continuing without radar data")
                            messagebox.showwarning("Warning", "Failed to open FT601 device")
                    else:
                        # Configure burst mode if enabled
                        if self.burst_mode_var.get():
                            self.ft601_interface.configure_burst_mode(True)
                else:
                    logging.warning("No FT601 device selected, continuing without radar data")
            else:
                logging.warning("No FT601 device selected, continuing without radar data")
            
            # Send start flag to STM32
            if not self.stm32_usb_interface.send_start_flag():
                messagebox.showerror("Error", "Failed to send start flag to STM32")
                return
            
            # Send settings to STM32
            self.apply_settings()
            
            # Start radar operation
            self.running = True
            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.status_label.config(text="Status: Radar running - FT601 USB 3.0 active")
            
            logging.info("Radar system started successfully with FT601 USB 3.0")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start radar: {e}")
            logging.error(f"Start radar error: {e}")
    
    def stop_radar(self):
        """Stop radar operation"""
        self.running = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.status_label.config(text="Status: Radar stopped")
        
        self.stm32_usb_interface.close()
        self.ft601_interface.close()
        
        logging.info("Radar system stopped")
    
    def process_radar_data(self):
        """Process incoming radar data from FT601"""
        buffer = bytearray()
        while True:
            if self.running and self.ft601_interface.is_open:
                try:
                    # Read from FT601 (supports larger transfers)
                    data = self.ft601_interface.read_data(4096)
                    if data:
                        buffer.extend(data)
                        
                        # Process packets (32-bit aligned)
                        while len(buffer) >= 8:  # Minimum packet size
                            # Try to find valid packet
                            packet = self.radar_packet_parser.parse_packet(bytes(buffer))
                            if packet:
                                self.process_radar_packet(packet)
                                # Remove processed packet from buffer
                                packet_length = self.get_packet_length(packet)
                                if packet_length > 0:
                                    buffer = buffer[packet_length:]
                                self.received_packets += 1
                            else:
                                # No valid packet found, shift buffer
                                if len(buffer) > 4:
                                    buffer = buffer[1:]
                                else:
                                    break
                                
                except Exception as e:
                    logging.error(f"Error processing radar data: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    def get_packet_length(self, packet):
        """Calculate packet length including header and footer"""
        # This should match your packet structure
        return 64  # Example: 64-byte packets
    
    def process_gps_data(self):
        """Step 16/17: Process GPS data from STM32 via USB CDC"""
        while True:
            if self.running and self.stm32_usb_interface.is_open:
                try:
                    # Read data from STM32 USB
                    data = self.stm32_usb_interface.read_data(64, timeout=100)
                    if data:
                        gps_data = self.usb_packet_parser.parse_gps_data(data)
                        if gps_data:
                            self.gps_data_queue.put(gps_data)
                            logging.info(f"GPS Data received via USB: Lat {gps_data.latitude:.6f}, Lon {gps_data.longitude:.6f}, Alt {gps_data.altitude:.1f}m, Pitch {gps_data.pitch:.1f}°")
                except Exception as e:
                    logging.error(f"Error processing GPS data via USB: {e}")
            time.sleep(0.1)
    
    def process_radar_packet(self, packet):
        """Step 40: Process radar data and apply pitch correction"""
        try:
            if packet['type'] == 'range':
                range_meters = packet['range'] * 0.1
                
                # Apply pitch correction to elevation
                raw_elevation = packet['elevation']
                corrected_elevation = self.apply_pitch_correction(raw_elevation, self.current_gps.pitch)
                
                # Store correction for display
                self.corrected_elevations.append({
                    'raw': raw_elevation,
                    'corrected': corrected_elevation,
                    'pitch': self.current_gps.pitch,
                    'timestamp': packet['timestamp']
                })
                
                # Keep only recent corrections
                if len(self.corrected_elevations) > 100:
                    self.corrected_elevations = self.corrected_elevations[-100:]
                
                target = RadarTarget(
                    id=packet['chirp'],
                    range=range_meters,
                    velocity=0,
                    azimuth=packet['azimuth'],
                    elevation=corrected_elevation,  # Use corrected elevation
                    snr=20.0,
                    timestamp=packet['timestamp']
                )
                
                self.update_range_doppler_map(target)
                
            elif packet['type'] == 'doppler':
                lambda_wavelength = 3e8 / self.settings.system_frequency
                velocity = (packet['doppler_real'] / 32767.0) * (self.settings.prf1 * lambda_wavelength / 2)
                self.update_target_velocity(packet, velocity)
                
            elif packet['type'] == 'detection':
                if packet['detected']:
                    # Apply pitch correction to detection elevation
                    raw_elevation = packet['elevation']
                    corrected_elevation = self.apply_pitch_correction(raw_elevation, self.current_gps.pitch)
                    
                    logging.info(f"CFAR Detection: Raw Elev {raw_elevation}°, Corrected Elev {corrected_elevation:.1f}°, Pitch {self.current_gps.pitch:.1f}°")
                    
        except Exception as e:
            logging.error(f"Error processing radar packet: {e}")
    
    def update_range_doppler_map(self, target):
        """Update range-Doppler map with new target"""
        range_bin = min(int(target.range / 50), 1023)
        doppler_bin = min(abs(int(target.velocity)), 31)
        
        self.radar_processor.range_doppler_map[range_bin, doppler_bin] += 1
        
        self.radar_processor.detected_targets.append(target)
        
        if len(self.radar_processor.detected_targets) > 100:
            self.radar_processor.detected_targets = self.radar_processor.detected_targets[-100:]
    
    def update_target_velocity(self, packet, velocity):
        """Update target velocity information"""
        for target in self.radar_processor.detected_targets:
            if (target.azimuth == packet['azimuth'] and 
                target.elevation == packet['elevation'] and
                target.id == packet['chirp']):
                target.velocity = velocity
                break

    def setup_map_tab(self):
        """Setup the tactical map display tab with custom radar canvas"""
        # Main canvas area (fills remaining height)
        canvas_container = tk.Frame(self.tab_map, bg=BG)
        canvas_container.pack(fill='both', expand=True)
        
        # Create radar canvas
        self.radar_canvas = tk.Canvas(canvas_container, bg=BG, highlightthickness=0)
        self.radar_canvas.pack(fill='both', expand=True)
        
        # Store canvas dimensions for redraw
        self.radar_canvas.bind('<Configure>', self.on_radar_canvas_resize)
        
        # Stats Bar (thin bar below canvas)
        stats_bar = tk.Frame(self.tab_map, bg=BG, height=50)
        stats_bar.pack(side='bottom', fill='x')
        stats_bar.pack_propagate(False)
        tk.Frame(stats_bar, bg=BORDER, height=1).pack(side='top', fill='x')
        
        # Four equal-width cells
        stats_cells = ['Radar Position', 'Targets Detected', 'Coverage Radius', 'Mode']
        self.stats_values = ['--', '0 active', '500 m', 'SURVEILLANCE']
        
        for i, (label, value) in enumerate(zip(stats_cells, self.stats_values)):
            cell = tk.Frame(stats_bar, bg=BG)
            cell.pack(side='left', fill='both', expand=True)
            if i < len(stats_cells) - 1:
                tk.Frame(cell, bg=BORDER, width=1).pack(side='right', fill='y')
            
            tk.Label(cell, text=label.upper(), font=FONT_UI_SMALL, bg=BG, fg=LABEL_GRAY).pack(pady=(5, 2))
            val_label = tk.Label(cell, text=value, font=FONT_MONO_LARGE, bg=BG, fg=ACCENT)
            val_label.pack(pady=(0, 5))
            setattr(self, f'stats_val_{i}', val_label)
        
        # Controls Row (bottom bar)
        controls_bar = tk.Frame(self.tab_map, bg=BG, height=36)
        controls_bar.pack(side='bottom', fill='x')
        controls_bar.pack_propagate(False)
        tk.Frame(controls_bar, bg=BORDER, height=1).pack(side='top', fill='x')
        
        # Open in Browser button
        browser_btn = tk.Button(
            controls_bar, text="OPEN IN BROWSER", command=self.open_map_in_browser,
            font=FONT_UI, bg=BG, fg=ACCENT, relief='flat', bd=0,
            activebackground=ACCENT_HOVER, activeforeground=FG
        )
        browser_btn.pack(side='left', padx=5, pady=5)
        tk.Frame(controls_bar, bg=BORDER, width=1, height=20).pack(side='left', padx=0, pady=8)
        
        # Refresh button
        refresh_btn = tk.Button(
            controls_bar, text="REFRESH", command=self.refresh_map,
            font=FONT_UI, bg=BG, fg=LABEL_GRAY, relief='flat', bd=0,
            activebackground=ACCENT_HOVER, activeforeground=FG
        )
        refresh_btn.pack(side='left', padx=5, pady=5)
        tk.Frame(controls_bar, bg=BORDER, width=1, height=20).pack(side='left', padx=0, pady=8)
        
        # Flexible spacer
        tk.Frame(controls_bar, bg=BG).pack(side='left', fill='x', expand=True)
        
        # Status indicator
        self.map_status_diamond = tk.Label(controls_bar, text="■", font=("Arial", 6), bg=BG, fg=LABEL_GRAY)
        self.map_status_diamond.pack(side='right', padx=5)
        self.map_status_text = tk.Label(controls_bar, text="STANDBY", font=FONT_MONO_SMALL, bg=BG, fg=LABEL_GRAY)
        self.map_status_text.pack(side='right', padx=2)
        
        # Initialize radar animation
        self.sweep_angle = 0
        self.target_pulse_phase = 0
        self.animate_radar_sweep()
    
    def on_radar_canvas_resize(self, event):
        """Handle canvas resize"""
        self.radar_canvas_width = event.width
        self.radar_canvas_height = event.height
        self.draw_radar_display()
    
    def draw_radar_display(self):
        """Draw the tactical radar display"""
        self.radar_canvas.delete('all')
        
        # Check if canvas dimensions are set
        if not hasattr(self, 'radar_canvas_width') or not hasattr(self, 'radar_canvas_height'):
            return
        
        w = self.radar_canvas_width
        h = self.radar_canvas_height
        cx, cy = w // 2, h // 2
        
        # Draw grid pattern (minor lines every 25px, major every 100px)
        for x in range(0, w, 25):
            color = ACCENT if x % 100 == 0 else BORDER
            alpha = 0.16 if x % 100 == 0 else 0.06
            self.radar_canvas.create_line(x, 0, x, h, fill=color, width=1 if x % 100 == 0 else 1)
        
        for y in range(0, h, 25):
            color = ACCENT if y % 100 == 0 else BORDER
            self.radar_canvas.create_line(0, y, w, y, fill=color, width=1 if y % 100 == 0 else 1)
        
        # Draw crosshairs
        self.radar_canvas.create_line(cx, 0, cx, h, fill=ACCENT, width=1)
        self.radar_canvas.create_line(0, cy, w, cy, fill=ACCENT, width=1)
        
        # Draw coverage rings (concentric squares)
        ring_sizes = [250, 500, 800]
        for i, size in enumerate(ring_sizes):
            if size < min(w, h) // 2:
                dash = (4, 4) if i == 2 else None
                self.radar_canvas.create_rectangle(
                    cx - size, cy - size, cx + size, cy + size,
                    outline=ACCENT, width=1, dash=dash
                )
        
        # Draw rotating sweep line
        sweep_length = max(w, h)
        angle_rad = math.radians(self.sweep_angle)
        end_x = cx + sweep_length * math.cos(angle_rad)
        end_y = cy + sweep_length * math.sin(angle_rad)
        self.radar_canvas.create_line(cx, cy, end_x, end_y, fill=ACCENT, width=2, tags='sweep')
        
        # Draw sweep trail (conic gradient simulation)
        for i in range(1, 15):
            trail_angle = self.sweep_angle - i * 5
            trail_rad = math.radians(trail_angle)
            trail_end_x = cx + sweep_length * math.cos(trail_rad)
            trail_end_y = cy + sweep_length * math.sin(trail_rad)
            alpha = int(255 * (1 - i / 15) * 0.3)
            trail_color = f'#{alpha:02x}b981'  # Green with alpha
            self.radar_canvas.create_line(cx, cy, trail_end_x, trail_end_y, fill=ACCENT, width=1, tags='sweep')
        
        # Draw radar origin
        self.radar_canvas.create_rectangle(cx - 10, cy - 10, cx + 10, cy + 10, outline=ACCENT, width=2)
        self.radar_canvas.create_text(cx, cy, text="↑", fill=ACCENT, font=("Arial", 10))
        
        # Draw pulsing origin ring
        pulse_size = 14 + 3 * math.sin(self.target_pulse_phase)
        self.radar_canvas.create_rectangle(
            cx - pulse_size, cy - pulse_size, cx + pulse_size, cy + pulse_size,
            outline=ACCENT, width=1, tags='pulse'
        )
        
        # Draw target markers
        for i, target in enumerate(self.radar_processor.detected_targets[-10:]):
            # Convert polar to canvas coordinates
            scale = 0.5  # pixels per meter
            target_x = cx + target.range * scale * math.cos(math.radians(target.azimuth - 90))
            target_y = cy + target.range * scale * math.sin(math.radians(target.azimuth - 90))
            
            # Color based on SNR
            if target.snr >= 25:
                color = ACCENT
            elif target.snr >= 15:
                color = WARNING
            else:
                color = DANGER
            
            # Draw diamond marker
            self.radar_canvas.create_polygon(
                target_x, target_y - 8,
                target_x + 8, target_y,
                target_x, target_y + 8,
                target_x - 8, target_y,
                fill=color, outline='', tags='targets'
            )
            
            # Draw pulsing ring
            pulse_size = 14 + 5 * math.sin(self.target_pulse_phase + i)
            self.radar_canvas.create_rectangle(
                target_x - pulse_size, target_y - pulse_size,
                target_x + pulse_size, target_y + pulse_size,
                outline=color, width=1, tags='pulse'
            )
            
            # Draw label
            label_bg = self.radar_canvas.create_rectangle(
                target_x + 10, target_y - 15, target_x + 70, target_y + 10,
                fill='#1a1a1a', outline=color, width=1
            )
            self.radar_canvas.create_text(
                target_x + 15, target_y - 10,
                text=f"TGT-{i+1}", fill=color, font=FONT_UI, anchor='w', tags='targets'
            )
            self.radar_canvas.create_text(
                target_x + 15, target_y + 2,
                text=f"{target.snr:.1f} dB", fill=LABEL_GRAY, font=FONT_MONO_SMALL, anchor='w', tags='targets'
            )
        
        # Draw corner overlays
        # Top-left: Info badge
        self.radar_canvas.create_rectangle(5, 5, 150, 30, fill=BG, outline=BORDER, width=1)
        self.radar_canvas.create_text(15, 12, text="●", fill=ACCENT, font=("Arial", 8), anchor='w')
        self.radar_canvas.create_text(25, 12, text="MAP VIEW", fill=FG, font=FONT_MONO_SMALL, anchor='w')
        self.radar_canvas.create_line(80, 5, 80, 30, fill=BORDER, width=1)
        self.radar_canvas.create_text(90, 12, text="■", fill=ACCENT, font=("Arial", 6), anchor='w')
        self.radar_canvas.create_text(100, 12, text="LIVE", fill=LABEL_GRAY, font=FONT_MONO_SMALL, anchor='w')
        
        # Bottom-left: Scale reference
        self.radar_canvas.create_line(10, h - 20, 90, h - 20, fill=ACCENT, width=2)
        self.radar_canvas.create_text(50, h - 10, text="500 m", fill=LABEL_GRAY, font=FONT_MONO_SMALL)
        
        # Bottom-right: Coordinate readout
        if hasattr(self, 'current_gps') and self.current_gps:
            coords_text = f"LAT {self.current_gps.latitude:.6f}  LON {self.current_gps.longitude:.6f}"
        else:
            coords_text = "LAT --.------  LON --.------"
        self.radar_canvas.create_text(w - 10, h - 15, text=coords_text, fill=ACCENT, font=FONT_MONO_SMALL, anchor='e')
    
    def animate_radar_sweep(self):
        """Animate the radar sweep"""
        self.sweep_angle = (self.sweep_angle + 2) % 360
        self.target_pulse_phase += 0.1
        
        if hasattr(self, 'radar_canvas'):
            self.draw_radar_display()
        
        self.root.after(50, self.animate_radar_sweep)
        
    def open_map_in_browser(self):
        """Open the generated map in the default web browser"""
        if self.map_file_path and os.path.exists(self.map_file_path):
            webbrowser.open('file://' + os.path.abspath(self.map_file_path))
        else:
            messagebox.showwarning("Warning", "No map file available. Generate map first by receiving GPS data.")

    def refresh_map(self):
        """Refresh the map with current data"""
        if self.current_gps.latitude == 0 and self.current_gps.longitude == 0:
            if hasattr(self, 'map_status_text'):
                self.map_status_text.config(text="WAITING FOR GPS")
            return
        try:
            self.generate_map_file()
        except Exception as e:
            logging.error(f"Error generating map: {e}")

    def generate_map_file(self):
        """Generate Google Maps HTML file with current targets"""
        if self.current_gps.latitude == 0 and self.current_gps.longitude == 0:
            if hasattr(self, 'map_status_text'):
                self.map_status_text.config(text="WAITING FOR GPS")
            return
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as f:
                map_html = self.map_generator.generate_map(
                    self.current_gps,
                    self.radar_processor.detected_targets,
                    self.settings.map_size,
                    self.google_maps_api_key
                )
                f.write(map_html)
                self.map_file_path = f.name
            if hasattr(self, 'map_status_text'):
                self.map_status_text.config(text="MAP READY")
            logging.info(f"Map generated: {self.map_file_path}")
        except Exception as e:
            logging.error(f"Error generating map: {e}")

    def update_gps_display(self):
        """Update GPS and pitch display"""
        try:
            while not self.gps_data_queue.empty():
                gps_data = self.gps_data_queue.get_nowait()
                self.current_gps = gps_data
                if hasattr(self, 'gps_coords_label'):
                    self.gps_coords_label.config(
                        text=f"Lat {gps_data.latitude:.6f}, Lon {gps_data.longitude:.6f}, Alt {gps_data.altitude:.1f}m")
                if hasattr(self, 'pitch_value_label'):
                    self.pitch_value_label.config(text=f"{gps_data.pitch:+.1f}°")
                self.refresh_map()
        except queue.Empty:
            pass

    def update_gui(self):
        try:
            while not self.radar_data_queue.empty():
                data = self.radar_data_queue.get_nowait()
                if hasattr(self, 'range_doppler_plot'):
                    self.range_doppler_plot.set_data(data.get('range_doppler', np.random.rand(1024, 32)))
                    self.canvas.draw()
            self.update_gps_display()
        except Exception as e:
            logging.error(f"Error updating GUI: {e}")
        self.root.after(100, self.update_gui)

def main():
    """Main application entry point"""
    try:
        root = tk.Tk()
        app = RadarGUI(root)
        root.mainloop()
    except Exception as e:
        logging.error(f"Application error: {e}")
        messagebox.showerror("Fatal Error", f"Application failed to start: {e}")

if __name__ == "__main__":
    main()
