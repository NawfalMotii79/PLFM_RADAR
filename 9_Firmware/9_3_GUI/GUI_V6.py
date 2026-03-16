import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
import struct
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

# Dark theme colors
DARK_BG = "#2b2b2b"
DARK_FG = "#e0e0e0"
DARK_ACCENT = "#3c3f41"
DARK_HIGHLIGHT = "#4e5254"
DARK_BORDER = "#555555"
DARK_TEXT = "#cccccc"
DARK_BUTTON = "#3c3f41"
DARK_BUTTON_HOVER = "#4e5254"
DARK_TREEVIEW = "#3c3f41"
DARK_TREEVIEW_ALT = "#404040"

RADAR_SETTINGS_LIMITS = {
    'system_frequency': (1e9, 100e9),
    'chirp_duration_1': (1e-6, 1000e-6),
    'chirp_duration_2': (0.1e-6, 10e-6),
    'chirps_per_position': (1, 256),
    'freq_min': (1e6, 100e6),
    'freq_max': (1e6, 100e6),
    'prf1': (100, 10000),
    'prf2': (100, 10000),
    'max_distance': (100, 100000),
    'map_size': (1000, 200000),
}
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
                                <p>Coverage: {coverage_radius/1000:.1f}km</p>
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
                                    <h3>Target #{target.id}</h3>
                                    <p>Range: ${{target.range:.1f}}m</p>
                                    <p>Velocity: ${{target.velocity:.1f}}m/s</p>
                                    <p>Azimuth: ${{target.azimuth}}°</p>
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
            targets_json = str(map_targets).replace("'", '"')
            targets_script = f"updateTargets({targets_json});"
        
        # Fill template
        map_html = self.map_html_template.format(
            lat=gps_data.latitude,
            lon=gps_data.longitude,
            alt=gps_data.altitude,
            pitch=gps_data.pitch,
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
        
        # Convert range to angular distance
        angular_distance = range_m / earth_radius
        
        # Convert to geographic coordinates
        target_lat = radar_lat + math.cos(azimuth_rad) * angular_distance * (180 / math.pi)
        target_lon = radar_lon + math.sin(azimuth_rad) * angular_distance * (180 / math.pi) / math.cos(math.radians(radar_lat))
        
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


class RadarGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Radar System GUI - FT601 USB 3.0")
        self.root.geometry("1400x900")
        
        # Apply dark theme
        self.root.configure(bg=DARK_BG)
        
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
        self.start_background_threads()

    def configure_dark_theme(self):
        """Configure ttk style for dark mercury theme"""
        self.style.configure('.', 
                           background=DARK_BG,
                           foreground=DARK_FG,
                           fieldbackground=DARK_ACCENT,
                           selectbackground=DARK_HIGHLIGHT,
                           selectforeground=DARK_FG,
                           troughcolor=DARK_ACCENT,
                           borderwidth=1,
                           focuscolor=DARK_BORDER)
        
        # Configure specific widgets
        self.style.configure('TFrame', background=DARK_BG)
        self.style.configure('TLabel', background=DARK_BG, foreground=DARK_FG)
        self.style.configure('TButton', 
                           background=DARK_BUTTON, 
                           foreground=DARK_FG,
                           borderwidth=1,
                           focuscolor=DARK_BORDER)
        self.style.map('TButton',
                      background=[('active', DARK_BUTTON_HOVER),
                                ('pressed', DARK_HIGHLIGHT)])
        
        self.style.configure('TCombobox', 
                           fieldbackground=DARK_ACCENT,
                           background=DARK_BG,
                           foreground=DARK_FG,
                           arrowcolor=DARK_FG)
        self.style.map('TCombobox',
                      fieldbackground=[('readonly', DARK_ACCENT)],
                      selectbackground=[('readonly', DARK_HIGHLIGHT)],
                      selectforeground=[('readonly', DARK_FG)])
        
        self.style.configure('TNotebook', background=DARK_BG, borderwidth=0)
        self.style.configure('TNotebook.Tab', 
                           background=DARK_ACCENT,
                           foreground=DARK_FG,
                           padding=[10, 5])
        self.style.map('TNotebook.Tab',
                      background=[('selected', DARK_HIGHLIGHT),
                                ('active', DARK_BUTTON_HOVER)])
        
        self.style.configure('Treeview',
                           background=DARK_TREEVIEW,
                           foreground=DARK_FG,
                           fieldbackground=DARK_TREEVIEW,
                           borderwidth=0)
        self.style.map('Treeview',
                      background=[('selected', DARK_HIGHLIGHT)])
        
        self.style.configure('Treeview.Heading',
                           background=DARK_ACCENT,
                           foreground=DARK_FG,
                           relief='flat')
        self.style.map('Treeview.Heading',
                      background=[('active', DARK_BUTTON_HOVER)])
        
        self.style.configure('TEntry',
                           fieldbackground=DARK_ACCENT,
                           foreground=DARK_FG,
                           insertcolor=DARK_FG)
        
        self.style.configure('Vertical.TScrollbar',
                           background=DARK_ACCENT,
                           troughcolor=DARK_BG,
                           borderwidth=0,
                           arrowsize=12)
        self.style.configure('Horizontal.TScrollbar',
                           background=DARK_ACCENT,
                           troughcolor=DARK_BG,
                           borderwidth=0,
                           arrowsize=12)
        
        self.style.configure('TLabelFrame', 
                           background=DARK_BG,
                           foreground=DARK_FG,
                           bordercolor=DARK_BORDER)
        self.style.configure('TLabelFrame.Label', 
                           background=DARK_BG,
                           foreground=DARK_FG)
    
    def create_gui(self):
        """Create the main GUI with tabs"""
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_map = ttk.Frame(self.notebook)
        self.tab_diagnostics = ttk.Frame(self.notebook)
        self.tab_settings = ttk.Frame(self.notebook)
        
        self.notebook.add(self.tab_main, text='Main View')
        self.notebook.add(self.tab_map, text='Map View')
        self.notebook.add(self.tab_diagnostics, text='Diagnostics')
        self.notebook.add(self.tab_settings, text='Settings')
        
        self.setup_main_tab()
        self.setup_map_tab()
        self.setup_settings_tab()
    
    def setup_main_tab(self):
        """Setup the main radar display tab"""
        # Control frame
        control_frame = ttk.Frame(self.tab_main)
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # USB Device selection
        ttk.Label(control_frame, text="STM32 USB Device:").grid(row=0, column=0, padx=5)
        self.stm32_usb_combo = ttk.Combobox(control_frame, state="readonly", width=40)
        self.stm32_usb_combo.grid(row=0, column=1, padx=5)
        
        # FT601 Device selection (replaces FTDI)
        ttk.Label(control_frame, text="FT601 USB 3.0 Device:").grid(row=0, column=2, padx=5)
        self.ft601_combo = ttk.Combobox(control_frame, state="readonly", width=40)
        self.ft601_combo.grid(row=0, column=3, padx=5)
        
        # Burst mode checkbox (new for FT601)
        self.burst_mode_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="Burst Mode", 
                       variable=self.burst_mode_var).grid(row=0, column=4, padx=5)
        
        ttk.Button(control_frame, text="Refresh Devices", 
                  command=self.refresh_devices).grid(row=0, column=5, padx=5)
        
        self.start_button = ttk.Button(control_frame, text="Start Radar", 
                                      command=self.start_radar)
        self.start_button.grid(row=0, column=6, padx=5)
        
        self.stop_button = ttk.Button(control_frame, text="Stop Radar", 
                                     command=self.stop_radar, state="disabled")
        self.stop_button.grid(row=0, column=7, padx=5)
        
        # GPS and Pitch info
        self.gps_label = ttk.Label(control_frame, text="GPS: Waiting for data...")
        self.gps_label.grid(row=1, column=0, columnspan=4, sticky='w', padx=5, pady=2)
        
        # Pitch display
        self.pitch_label = ttk.Label(control_frame, text="Pitch: --.--°")
        self.pitch_label.grid(row=1, column=4, columnspan=2, padx=5, pady=2)
        
        # Status info with FT601 specific info
        self.status_label = ttk.Label(control_frame, 
                                      text="Status: Ready - FT601 USB 3.0")
        self.status_label.grid(row=1, column=6, columnspan=2, sticky='e', padx=5, pady=2)
        
        # [Rest of setup_main_tab remains the same]
        # ...
    
    def setup_map_tab(self):
        """Setup the map display tab with Google Maps"""
        map_frame = ttk.Frame(self.tab_map)
        map_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Map controls
        controls_frame = ttk.Frame(map_frame)
        controls_frame.pack(fill='x', pady=5)
        
        ttk.Button(controls_frame, text="Open Map in Browser", 
                  command=self.open_map_in_browser).pack(side='left', padx=5)
        
        ttk.Button(controls_frame, text="Refresh Map", 
                  command=self.refresh_map).pack(side='left', padx=5)
        
        self.map_status_label = ttk.Label(controls_frame, text="Map: Ready to generate")
        self.map_status_label.pack(side='left', padx=20)
        
        # Map info display
        info_frame = ttk.Frame(map_frame)
        info_frame.pack(fill='x', pady=5)
        
        self.map_info_label = ttk.Label(info_frame, text="No GPS data received yet", font=('Arial', 10))
        self.map_info_label.pack()
    
    def setup_settings_tab(self):
        """Setup the settings tab with additional chirp durations and map size"""
        settings_frame = ttk.Frame(self.tab_settings)
        settings_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        entries = [
            ('System Frequency (Hz):', 'system_frequency', 10e9),
            ('Chirp Duration 1 - Long (s):', 'chirp_duration_1', 30e-6),
            ('Chirp Duration 2 - Short (s):', 'chirp_duration_2', 0.5e-6),
            ('Chirps per Position:', 'chirps_per_position', 32),
            ('Frequency Min (Hz):', 'freq_min', 10e6),
            ('Frequency Max (Hz):', 'freq_max', 30e6),
            ('PRF1 (Hz):', 'prf1', 1000),
            ('PRF2 (Hz):', 'prf2', 2000),
            ('Max Distance (m):', 'max_distance', 50000),
            ('Map Size (m):', 'map_size', 50000),
            ('Google Maps API Key:', 'google_maps_api_key', 'YOUR_GOOGLE_MAPS_API_KEY')
        ]
        
        self.settings_vars = {}
        
        for i, (label, attr, default) in enumerate(entries):
            ttk.Label(settings_frame, text=label).grid(row=i, column=0, sticky='w', padx=5, pady=5)
            var = tk.StringVar(value=str(default))
            entry = ttk.Entry(settings_frame, textvariable=var, width=25)
            entry.grid(row=i, column=1, padx=5, pady=5)
            self.settings_vars[attr] = var
        
        ttk.Button(settings_frame, text="Apply Settings", 
                  command=self.apply_settings).grid(row=len(entries), column=0, columnspan=2, pady=10)

    def _parse_settings_from_form(self):
        """Read settings from the UI and return a validated RadarSettings instance."""
        parsed_settings = RadarSettings(
            system_frequency=float(self.settings_vars['system_frequency'].get()),
            chirp_duration_1=float(self.settings_vars['chirp_duration_1'].get()),
            chirp_duration_2=float(self.settings_vars['chirp_duration_2'].get()),
            chirps_per_position=int(self.settings_vars['chirps_per_position'].get()),
            freq_min=float(self.settings_vars['freq_min'].get()),
            freq_max=float(self.settings_vars['freq_max'].get()),
            prf1=float(self.settings_vars['prf1'].get()),
            prf2=float(self.settings_vars['prf2'].get()),
            max_distance=float(self.settings_vars['max_distance'].get()),
            map_size=float(self.settings_vars['map_size'].get()),
        )

        self._validate_radar_settings(parsed_settings)
        return parsed_settings

    def _validate_radar_settings(self, settings):
        """Mirror the firmware-side range checks before sending settings to STM32."""
        for field_name, (minimum, maximum) in RADAR_SETTINGS_LIMITS.items():
            value = getattr(settings, field_name)
            if value < minimum or value > maximum:
                raise ValueError(
                    f"{field_name} must be between {minimum:g} and {maximum:g}."
                )

        if settings.freq_max <= settings.freq_min:
            raise ValueError("freq_max must be greater than freq_min.")

        return True
    
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
        
        if stm32_names:
            self.stm32_usb_combo.current(0)
        if ft601_names:
            self.ft601_combo.current(0)
    
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
    
    def apply_settings(self):
        """Step 13: Apply and send radar settings via USB"""
        try:
            parsed_settings = self._parse_settings_from_form()
            self.google_maps_api_key = self.settings_vars['google_maps_api_key'].get()

            self.settings = parsed_settings

            if self.stm32_usb_interface.is_open:
                if not self.stm32_usb_interface.send_settings(self.settings):
                    messagebox.showerror("Error", "Failed to send settings to STM32 via USB")
                    logging.error("Radar settings validation passed, but USB send failed")
                    return

                messagebox.showinfo("Success", "Settings applied and sent to STM32 via USB")
                logging.info("Radar settings applied and sent via USB")
            else:
                messagebox.showinfo("Success", "Settings applied locally")
                logging.info("Radar settings applied locally; STM32 USB is not connected")

        except ValueError as e:
            messagebox.showerror("Error", f"Invalid setting value: {e}")
    
    def start_background_threads(self):
        """Start background data processing threads"""
        self.radar_thread = threading.Thread(target=self.process_radar_data, daemon=True)
        self.radar_thread.start()
        
        self.gps_thread = threading.Thread(target=self.process_gps_data, daemon=True)
        self.gps_thread.start()
        
        self.root.after(100, self.update_gui)

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
    
    def get_packet_length(self, packet):
        """Calculate packet length including header and footer"""
        # This should match your packet structure
        return 64  # Example: 64-byte packets
    

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
    
    def open_map_in_browser(self):
        """Open the generated map in the default web browser"""
        if self.map_file_path and os.path.exists(self.map_file_path):
            webbrowser.open('file://' + os.path.abspath(self.map_file_path))
        else:
            messagebox.showwarning("Warning", "No map file available. Generate map first by receiving GPS data.")
    
    def refresh_map(self):
        """Refresh the map with current data"""
        self.generate_map()
    
    def generate_map(self):
        """Generate Google Maps HTML file with current targets"""
        if self.current_gps.latitude == 0 and self.current_gps.longitude == 0:
            self.map_status_label.config(text="Map: Waiting for GPS data")
            return
        
        try:
            # Create temporary HTML file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as f:
                map_html = self.map_generator.generate_map(
                    self.current_gps,
                    self.radar_processor.detected_targets,
                    self.settings.map_size,
                    self.google_maps_api_key
                )
                f.write(map_html)
                self.map_file_path = f.name
            
            self.map_status_label.config(text=f"Map: Generated at {self.map_file_path}")
            self.map_info_label.config(
                text=f"Radar: {self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f} | "
                     f"Targets: {len(self.radar_processor.detected_targets)} | "
                     f"Coverage: {self.settings.map_size/1000:.1f}km"
            )
            logging.info(f"Map generated: {self.map_file_path}")
            
        except Exception as e:
            logging.error(f"Error generating map: {e}")
            self.map_status_label.config(text=f"Map: Error - {str(e)}")
    
    def update_gps_display(self):
        """Step 18: Update GPS and pitch display"""
        try:
            while not self.gps_data_queue.empty():
                gps_data = self.gps_data_queue.get_nowait()
                self.current_gps = gps_data
                
                # Update GPS label
                self.gps_label.config(
                    text=f"GPS: Lat {gps_data.latitude:.6f}, Lon {gps_data.longitude:.6f}, Alt {gps_data.altitude:.1f}m")
                
                # Update pitch label with color coding
                pitch_text = f"Pitch: {gps_data.pitch:+.1f}°"
                self.pitch_label.config(text=pitch_text)
                
                # Color code based on pitch magnitude
                if abs(gps_data.pitch) > 10:
                    self.pitch_label.config(foreground='red')  # High pitch warning
                elif abs(gps_data.pitch) > 5:
                    self.pitch_label.config(foreground='orange')  # Medium pitch
                else:
                    self.pitch_label.config(foreground='green')  # Normal pitch
                
                # Generate/update map when new GPS data arrives
                self.generate_map()
                
        except queue.Empty:
            pass
    
    def update_targets_list(self):
        """Update the targets list display with corrected elevations"""
        for item in self.targets_tree.get_children():
            self.targets_tree.delete(item)
        
        for target in self.radar_processor.detected_targets[-20:]:
            # Find the corresponding raw elevation if available
            raw_elevation = "N/A"
            for correction in self.corrected_elevations[-20:]:
                if abs(correction['corrected'] - target.elevation) < 0.1:  # Fuzzy match
                    raw_elevation = f"{correction['raw']}"
                    break
            
            self.targets_tree.insert('', 'end', values=(
                target.track_id, 
                f"{target.range:.1f}", 
                f"{target.velocity:.1f}",
                target.azimuth,
                raw_elevation,  # Show raw elevation
                f"{target.elevation:.1f}",  # Show corrected elevation
                f"{target.snr:.1f}"
            ))

    
    def update_gui(self):
        """Step 40: Update all GUI displays"""
        try:
            # Update status with pitch information
            if self.running:
                self.status_label.config(
                    text=f"Status: Running - Packets: {self.received_packets} - Pitch: {self.current_gps.pitch:+.1f}°")
            
            # Update range-Doppler map
            if hasattr(self, 'range_doppler_plot'):
                display_data = np.log10(self.radar_processor.range_doppler_map + 1)
                self.range_doppler_plot.set_array(display_data)
                self.canvas.draw_idle()
            
            # Update targets list
            self.update_targets_list()
            
            # Update GPS and pitch display
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
