"""
v7 — PLFM Radar GUI V7 (PyQt6 edition).

Re-exports all public classes and functions from sub-modules for convenient
top-level imports:

    from v7 import RadarDashboard, RadarTarget, RadarSettings, ...
"""

# Models / constants
# Main dashboard
from .dashboard import (
    RadarDashboard,
    RangeDopplerCanvas,
)

# Hardware interfaces
from .hardware import (
    FT2232HQInterface,
    STM32USBInterface,
)

# Map widget
from .map_widget import (
    MapBridge,
    RadarMapWidget,
)
from .models import (
    CRCMOD_AVAILABLE,
    DARK_ACCENT,
    DARK_BG,
    DARK_BORDER,
    DARK_BUTTON,
    DARK_BUTTON_HOVER,
    DARK_ERROR,
    DARK_FG,
    DARK_HIGHLIGHT,
    DARK_INFO,
    DARK_SUCCESS,
    DARK_TEXT,
    DARK_TREEVIEW,
    DARK_TREEVIEW_ALT,
    DARK_WARNING,
    FILTERPY_AVAILABLE,
    FTDI_AVAILABLE,
    SCIPY_AVAILABLE,
    SKLEARN_AVAILABLE,
    USB_AVAILABLE,
    GPSData,
    ProcessingConfig,
    RadarSettings,
    RadarTarget,
    TileServer,
)

# Processing pipeline
from .processing import (
    RadarPacketParser,
    RadarProcessor,
    USBPacketParser,
    apply_pitch_correction,
)

# Workers and simulator
from .workers import (
    GPSDataWorker,
    RadarDataWorker,
    TargetSimulator,
    polar_to_geographic,
)

__all__ = [
    # models
    "RadarTarget", "RadarSettings", "GPSData", "ProcessingConfig", "TileServer",
    "DARK_BG", "DARK_FG", "DARK_ACCENT", "DARK_HIGHLIGHT", "DARK_BORDER",
    "DARK_TEXT", "DARK_BUTTON", "DARK_BUTTON_HOVER",
    "DARK_TREEVIEW", "DARK_TREEVIEW_ALT",
    "DARK_SUCCESS", "DARK_WARNING", "DARK_ERROR", "DARK_INFO",
    "USB_AVAILABLE", "FTDI_AVAILABLE", "SCIPY_AVAILABLE",
    "SKLEARN_AVAILABLE", "FILTERPY_AVAILABLE", "CRCMOD_AVAILABLE",
    # hardware
    "FT2232HQInterface", "STM32USBInterface",
    # processing
    "RadarProcessor", "RadarPacketParser", "USBPacketParser",
    "apply_pitch_correction",
    # workers
    "RadarDataWorker", "GPSDataWorker", "TargetSimulator",
    "polar_to_geographic",
    # map
    "MapBridge", "RadarMapWidget",
    # dashboard
    "RadarDashboard", "RangeDopplerCanvas",
]
