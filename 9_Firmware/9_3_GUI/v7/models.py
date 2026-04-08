"""
v7.models — Data classes, enums, and theme constants for the PLFM Radar GUI V7.

This module defines the core data structures used throughout the application:
  - RadarTarget, RadarSettings, GPSData (dataclasses)
  - TileServer (enum for map tile providers)
  - Dark theme color constants
  - Optional dependency availability flags
"""

import logging
from dataclasses import asdict, dataclass
from enum import Enum

# ---------------------------------------------------------------------------
# Optional dependency flags (graceful degradation)
# ---------------------------------------------------------------------------
try:
    import usb.core
    import usb.util  # noqa: F401 — availability check
    USB_AVAILABLE = True
except ImportError:
    USB_AVAILABLE = False
    logging.warning("pyusb not available. USB functionality will be disabled.")

try:
    from pyftdi.ftdi import (
        Ftdi,  # noqa: F401 — availability check
        FtdiError,  # noqa: F401 — availability check
    )
    from pyftdi.usbtools import UsbTools  # noqa: F401 — availability check
    FTDI_AVAILABLE = True
except ImportError:
    FTDI_AVAILABLE = False
    logging.warning("pyftdi not available. FTDI functionality will be disabled.")

try:
    from scipy import signal as _scipy_signal  # noqa: F401 — availability check
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    logging.warning("scipy not available. Some DSP features will be disabled.")

try:
    from sklearn.cluster import DBSCAN as _DBSCAN  # noqa: F401 — availability check
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    logging.warning("sklearn not available. Clustering will be disabled.")

try:
    from filterpy.kalman import KalmanFilter as _KalmanFilter  # noqa: F401 — availability check
    FILTERPY_AVAILABLE = True
except ImportError:
    FILTERPY_AVAILABLE = False
    logging.warning("filterpy not available. Kalman tracking will be disabled.")

try:
    import crcmod as _crcmod  # noqa: F401 — availability check
    CRCMOD_AVAILABLE = True
except ImportError:
    CRCMOD_AVAILABLE = False
    logging.warning("crcmod not available. CRC validation will use fallback.")

# ---------------------------------------------------------------------------
# Dark theme color constants (shared by all modules)
# ---------------------------------------------------------------------------
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
DARK_SUCCESS = "#4CAF50"
DARK_WARNING = "#FFC107"
DARK_ERROR = "#F44336"
DARK_INFO = "#2196F3"

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class RadarTarget:
    """Represents a detected radar target."""
    id: int
    range: float           # Range in meters
    velocity: float        # Velocity in m/s (positive = approaching)
    azimuth: float         # Azimuth angle in degrees
    elevation: float       # Elevation angle in degrees
    latitude: float = 0.0
    longitude: float = 0.0
    snr: float = 0.0       # Signal-to-noise ratio in dB
    timestamp: float = 0.0
    track_id: int = -1
    classification: str = "unknown"

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)


@dataclass
class RadarSettings:
    """Radar system configuration parameters."""
    system_frequency: float = 10e9      # Hz
    chirp_duration_1: float = 30e-6     # Long chirp duration (s)
    chirp_duration_2: float = 0.5e-6    # Short chirp duration (s)
    chirps_per_position: int = 32
    freq_min: float = 10e6              # Hz
    freq_max: float = 30e6              # Hz
    prf1: float = 1000                  # PRF 1 (Hz)
    prf2: float = 2000                  # PRF 2 (Hz)
    max_distance: float = 50000         # Max detection range (m)
    map_size: float = 50000             # Map display size (m)
    coverage_radius: float = 50000      # Map coverage radius (m)


@dataclass
class GPSData:
    """GPS position and orientation data."""
    latitude: float
    longitude: float
    altitude: float
    pitch: float            # Pitch angle in degrees
    heading: float = 0.0    # Heading in degrees (0 = North)
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


# ---------------------------------------------------------------------------
# Tile server enum
# ---------------------------------------------------------------------------

@dataclass
class ProcessingConfig:
    """Signal processing pipeline configuration.

    Controls: MTI filter, CFAR detector, DC notch removal,
    windowing, detection threshold, DBSCAN clustering, and Kalman tracking.
    """

    # MTI (Moving Target Indication)
    mti_enabled: bool = False
    mti_order: int = 2                     # 1, 2, or 3

    # CFAR (Constant False Alarm Rate)
    cfar_enabled: bool = False
    cfar_type: str = "CA-CFAR"             # CA-CFAR, OS-CFAR, GO-CFAR, SO-CFAR
    cfar_guard_cells: int = 2
    cfar_training_cells: int = 8
    cfar_threshold_factor: float = 5.0     # PFA-related scalar

    # DC Notch / DC Removal
    dc_notch_enabled: bool = False

    # Windowing (applied before FFT)
    window_type: str = "Hann"              # None, Hann, Hamming, Blackman, Kaiser, Chebyshev

    # Detection threshold (dB above noise floor)
    detection_threshold_db: float = 12.0

    # DBSCAN Clustering
    clustering_enabled: bool = True
    clustering_eps: float = 100.0
    clustering_min_samples: int = 2

    # Kalman Tracking
    tracking_enabled: bool = True


# ---------------------------------------------------------------------------
# Tile server enum
# ---------------------------------------------------------------------------

class TileServer(Enum):
    """Available map tile servers."""
    OPENSTREETMAP = "osm"
    GOOGLE_MAPS = "google"
    GOOGLE_SATELLITE = "google_sat"
    GOOGLE_HYBRID = "google_hybrid"
    ESRI_SATELLITE = "esri_sat"
