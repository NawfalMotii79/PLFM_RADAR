"""
v7.dashboard — Main application window for the PLFM Radar GUI V7.

RadarDashboard is a QMainWindow with four tabs:
  1. Main View   — Range-Doppler matplotlib canvas, device combos, Start/Stop, targets table
  2. Map View    — Embedded Leaflet map + sidebar (position, coverage, demo, target info)
  3. Diagnostics — Connection indicators, packet stats, dependency status, log viewer
  4. Settings    — All radar parameters + About section

Integrates: hardware interfaces, QThread workers, TargetSimulator, RadarMapWidget.
"""

import logging
import time
from typing import List, Optional

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt6.QtCore import Qt, QTimer, pyqtSlot
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from .hardware import FT2232HQInterface, STM32USBInterface
from .map_widget import RadarMapWidget
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
)
from .processing import RadarPacketParser, RadarProcessor, USBPacketParser
from .workers import GPSDataWorker, RadarDataWorker, TargetSimulator

logger = logging.getLogger(__name__)


# =============================================================================
# Range-Doppler Canvas (matplotlib)
# =============================================================================

class RangeDopplerCanvas(FigureCanvasQTAgg):
    """Matplotlib canvas showing the Range-Doppler map with dark theme."""

    def __init__(self, parent=None):
        fig = Figure(figsize=(10, 6), facecolor=DARK_BG)
        self.ax = fig.add_subplot(111, facecolor=DARK_ACCENT)

        self._data = np.zeros((1024, 32))
        self.im = self.ax.imshow(
            self._data, aspect="auto", cmap="hot",
            extent=[0, 32, 0, 1024], origin="lower",
        )

        self.ax.set_title("Range-Doppler Map (Pitch Corrected)", color=DARK_FG)
        self.ax.set_xlabel("Doppler Bin", color=DARK_FG)
        self.ax.set_ylabel("Range Bin", color=DARK_FG)
        self.ax.tick_params(colors=DARK_FG)
        for spine in self.ax.spines.values():
            spine.set_color(DARK_BORDER)

        fig.tight_layout()
        super().__init__(fig)

    def update_map(self, rdm: np.ndarray):
        display = np.log10(rdm + 1)
        self.im.set_data(display)
        self.im.set_clim(vmin=display.min(), vmax=max(display.max(), 0.1))
        self.draw_idle()


# =============================================================================
# RadarDashboard — main window
# =============================================================================

class RadarDashboard(QMainWindow):
    """Main application window with 4 tabs."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("PLFM Radar System GUI V7 — PyQt6")
        self.setGeometry(100, 60, 1500, 950)

        # ---- Core objects --------------------------------------------------
        self._settings = RadarSettings()
        self._radar_position = GPSData(
            latitude=41.9028, longitude=12.4964,
            altitude=0.0, pitch=0.0, heading=0.0, timestamp=0.0,
        )

        # Hardware interfaces
        self._stm32 = STM32USBInterface()
        self._ft2232hq = FT2232HQInterface()

        # Processing
        self._processor = RadarProcessor()
        self._radar_parser = RadarPacketParser()
        self._usb_parser = USBPacketParser()
        self._processing_config = ProcessingConfig()

        # Device lists (cached for index lookup)
        self._stm32_devices: list = []
        self._ft2232hq_devices: list = []

        # Workers (created on demand)
        self._radar_worker: Optional[RadarDataWorker] = None
        self._gps_worker: Optional[GPSDataWorker] = None
        self._simulator: Optional[TargetSimulator] = None

        # State
        self._running = False
        self._demo_mode = False
        self._start_time = time.time()
        self._radar_stats: dict = {}
        self._gps_packet_count = 0
        self._current_targets: List[RadarTarget] = []
        self._corrected_elevations: list = []

        # ---- Build UI ------------------------------------------------------
        self._apply_dark_theme()
        self._setup_ui()
        self._setup_statusbar()

        # GUI refresh timer (100 ms)
        self._gui_timer = QTimer(self)
        self._gui_timer.timeout.connect(self._refresh_gui)
        self._gui_timer.start(100)

        # Log handler for diagnostics
        self._log_handler = _QtLogHandler(self._log_append)
        self._log_handler.setLevel(logging.INFO)
        logging.getLogger().addHandler(self._log_handler)

        logger.info("RadarDashboard initialised")

    # =====================================================================
    # Dark theme
    # =====================================================================

    def _apply_dark_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {DARK_BG};
                color: {DARK_FG};
            }}
            QTabWidget::pane {{
                border: 1px solid {DARK_BORDER};
                background-color: {DARK_BG};
            }}
            QTabBar::tab {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                padding: 8px 18px;
                border: 1px solid {DARK_BORDER};
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: {DARK_HIGHLIGHT};
            }}
            QTabBar::tab:hover {{
                background-color: {DARK_BUTTON_HOVER};
            }}
            QGroupBox {{
                border: 1px solid {DARK_BORDER};
                border-radius: 4px;
                margin-top: 12px;
                padding-top: 12px;
                font-weight: bold;
                color: {DARK_FG};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }}
            QPushButton {{
                background-color: {DARK_BUTTON};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 6px 16px;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background-color: {DARK_BUTTON_HOVER};
            }}
            QPushButton:pressed {{
                background-color: {DARK_HIGHLIGHT};
            }}
            QPushButton:disabled {{
                color: {DARK_BORDER};
            }}
            QComboBox {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 4px 8px;
                border-radius: 4px;
            }}
            QLineEdit, QSpinBox, QDoubleSpinBox {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                padding: 4px 8px;
                border-radius: 4px;
            }}
            QCheckBox {{
                color: {DARK_FG};
                spacing: 6px;
            }}
            QLabel {{
                color: {DARK_FG};
            }}
            QTableWidget {{
                background-color: {DARK_TREEVIEW};
                alternate-background-color: {DARK_TREEVIEW_ALT};
                color: {DARK_FG};
                gridline-color: {DARK_BORDER};
                border: 1px solid {DARK_BORDER};
            }}
            QTableWidget::item:selected {{
                background-color: {DARK_INFO};
            }}
            QHeaderView::section {{
                background-color: {DARK_HIGHLIGHT};
                color: {DARK_FG};
                padding: 6px;
                border: none;
                border-right: 1px solid {DARK_BORDER};
                border-bottom: 1px solid {DARK_BORDER};
            }}
            QPlainTextEdit {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
                border: 1px solid {DARK_BORDER};
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }}
            QScrollBar:vertical {{
                background-color: {DARK_ACCENT};
                width: 12px;
            }}
            QScrollBar::handle:vertical {{
                background-color: {DARK_HIGHLIGHT};
                border-radius: 6px;
                min-height: 20px;
            }}
            QStatusBar {{
                background-color: {DARK_ACCENT};
                color: {DARK_FG};
            }}
        """)

    # =====================================================================
    # UI construction
    # =====================================================================

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        self._tabs = QTabWidget()
        main_layout.addWidget(self._tabs)

        self._create_main_tab()
        self._create_map_tab()
        self._create_diagnostics_tab()
        self._create_settings_tab()

    # -----------------------------------------------------------------
    # TAB 1: Main View
    # -----------------------------------------------------------------

    def _create_main_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)

        # ---- Control bar ---------------------------------------------------
        ctrl = QFrame()
        ctrl.setStyleSheet(f"background-color: {DARK_ACCENT}; border-radius: 4px;")
        ctrl_layout = QGridLayout(ctrl)
        ctrl_layout.setContentsMargins(8, 6, 8, 6)

        # Row 0: device combos & buttons
        ctrl_layout.addWidget(QLabel("STM32 USB:"), 0, 0)
        self._stm32_combo = QComboBox()
        self._stm32_combo.setMinimumWidth(200)
        ctrl_layout.addWidget(self._stm32_combo, 0, 1)

        ctrl_layout.addWidget(QLabel("FT2232HQ (Primary):"), 0, 2)
        self._ft2232hq_combo = QComboBox()
        self._ft2232hq_combo.setMinimumWidth(200)
        ctrl_layout.addWidget(self._ft2232hq_combo, 0, 3)

        refresh_btn = QPushButton("Refresh Devices")
        refresh_btn.clicked.connect(self._refresh_devices)
        ctrl_layout.addWidget(refresh_btn, 0, 4)

        self._start_btn = QPushButton("Start Radar")
        self._start_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_SUCCESS}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #66BB6A; }}"
        )
        self._start_btn.clicked.connect(self._start_radar)
        ctrl_layout.addWidget(self._start_btn, 0, 8)

        self._stop_btn = QPushButton("Stop Radar")
        self._stop_btn.setEnabled(False)
        self._stop_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_ERROR}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #EF5350; }}"
        )
        self._stop_btn.clicked.connect(self._stop_radar)
        ctrl_layout.addWidget(self._stop_btn, 0, 9)

        self._demo_btn_main = QPushButton("Start Demo")
        self._demo_btn_main.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_INFO}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #42A5F5; }}"
        )
        self._demo_btn_main.clicked.connect(self._toggle_demo_main)
        ctrl_layout.addWidget(self._demo_btn_main, 0, 10)

        # Row 1: status labels
        self._gps_label = QLabel("GPS: Waiting for data...")
        ctrl_layout.addWidget(self._gps_label, 1, 0, 1, 4)

        self._pitch_label = QLabel("Pitch: --.--\u00b0")
        ctrl_layout.addWidget(self._pitch_label, 1, 4, 1, 2)

        self._status_label_main = QLabel("Status: Ready")
        self._status_label_main.setAlignment(Qt.AlignmentFlag.AlignRight)
        ctrl_layout.addWidget(self._status_label_main, 1, 6, 1, 5)

        layout.addWidget(ctrl)

        # ---- Display area (range-doppler + targets table) ------------------
        display_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Range-Doppler canvas
        self._rdm_canvas = RangeDopplerCanvas()
        display_splitter.addWidget(self._rdm_canvas)

        # Targets table
        targets_group = QGroupBox("Detected Targets (Pitch Corrected)")
        tg_layout = QVBoxLayout(targets_group)

        self._targets_table_main = QTableWidget()
        self._targets_table_main.setColumnCount(7)
        self._targets_table_main.setHorizontalHeaderLabels([
            "Track ID", "Range (m)", "Velocity (m/s)",
            "Azimuth", "Raw Elev", "Corr Elev", "SNR (dB)",
        ])
        self._targets_table_main.setAlternatingRowColors(True)
        self._targets_table_main.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )
        header = self._targets_table_main.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        tg_layout.addWidget(self._targets_table_main)

        display_splitter.addWidget(targets_group)
        display_splitter.setSizes([800, 400])

        layout.addWidget(display_splitter, stretch=1)
        self._tabs.addTab(tab, "Main View")

    # -----------------------------------------------------------------
    # TAB 2: Map View
    # -----------------------------------------------------------------

    def _create_map_tab(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        layout.setContentsMargins(4, 4, 4, 4)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Map widget
        self._map_widget = RadarMapWidget(
            radar_lat=self._radar_position.latitude,
            radar_lon=self._radar_position.longitude,
        )
        self._map_widget.targetSelected.connect(self._on_target_selected)
        splitter.addWidget(self._map_widget)

        # Sidebar
        sidebar = QWidget()
        sidebar.setMaximumWidth(320)
        sidebar.setMinimumWidth(280)
        sb_layout = QVBoxLayout(sidebar)
        sb_layout.setContentsMargins(8, 8, 8, 8)

        # Radar position group
        pos_group = QGroupBox("Radar Position")
        pos_layout = QGridLayout(pos_group)

        self._lat_spin = QDoubleSpinBox()
        self._lat_spin.setRange(-90, 90)
        self._lat_spin.setDecimals(6)
        self._lat_spin.setValue(self._radar_position.latitude)
        self._lat_spin.valueChanged.connect(self._on_position_changed)

        self._lon_spin = QDoubleSpinBox()
        self._lon_spin.setRange(-180, 180)
        self._lon_spin.setDecimals(6)
        self._lon_spin.setValue(self._radar_position.longitude)
        self._lon_spin.valueChanged.connect(self._on_position_changed)

        self._alt_spin = QDoubleSpinBox()
        self._alt_spin.setRange(0, 50000)
        self._alt_spin.setDecimals(1)
        self._alt_spin.setValue(0.0)
        self._alt_spin.setSuffix(" m")

        pos_layout.addWidget(QLabel("Latitude:"), 0, 0)
        pos_layout.addWidget(self._lat_spin, 0, 1)
        pos_layout.addWidget(QLabel("Longitude:"), 1, 0)
        pos_layout.addWidget(self._lon_spin, 1, 1)
        pos_layout.addWidget(QLabel("Altitude:"), 2, 0)
        pos_layout.addWidget(self._alt_spin, 2, 1)

        sb_layout.addWidget(pos_group)

        # Coverage group
        cov_group = QGroupBox("Coverage")
        cov_layout = QGridLayout(cov_group)

        self._coverage_spin = QDoubleSpinBox()
        self._coverage_spin.setRange(1, 200)
        self._coverage_spin.setDecimals(1)
        self._coverage_spin.setValue(self._settings.coverage_radius / 1000)
        self._coverage_spin.setSuffix(" km")
        self._coverage_spin.valueChanged.connect(self._on_coverage_changed)

        cov_layout.addWidget(QLabel("Radius:"), 0, 0)
        cov_layout.addWidget(self._coverage_spin, 0, 1)

        sb_layout.addWidget(cov_group)

        # Demo controls group
        demo_group = QGroupBox("Demo Mode")
        demo_layout = QVBoxLayout(demo_group)

        self._demo_btn_map = QPushButton("Start Demo")
        self._demo_btn_map.setCheckable(True)
        self._demo_btn_map.clicked.connect(self._toggle_demo_map)
        demo_layout.addWidget(self._demo_btn_map)

        add_btn = QPushButton("Add Random Target")
        add_btn.clicked.connect(self._add_demo_target)
        demo_layout.addWidget(add_btn)

        sb_layout.addWidget(demo_group)

        # Selected target info
        info_group = QGroupBox("Selected Target")
        info_layout = QVBoxLayout(info_group)

        self._target_info_label = QLabel("No target selected")
        self._target_info_label.setWordWrap(True)
        self._target_info_label.setStyleSheet(f"color: {DARK_TEXT}; padding: 8px;")
        info_layout.addWidget(self._target_info_label)

        sb_layout.addWidget(info_group)
        sb_layout.addStretch()

        splitter.addWidget(sidebar)
        splitter.setSizes([900, 300])

        layout.addWidget(splitter)
        self._tabs.addTab(tab, "Map View")

    # -----------------------------------------------------------------
    # TAB 3: Diagnostics
    # -----------------------------------------------------------------

    def _create_diagnostics_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(8, 8, 8, 8)

        top_row = QHBoxLayout()

        # Connection status
        conn_group = QGroupBox("Connection Status")
        conn_layout = QGridLayout(conn_group)

        self._conn_stm32 = self._make_status_label("STM32 USB")
        self._conn_ft2232hq = self._make_status_label("FT2232HQ (Primary)")

        conn_layout.addWidget(QLabel("STM32 USB:"), 0, 0)
        conn_layout.addWidget(self._conn_stm32, 0, 1)
        conn_layout.addWidget(QLabel("FT2232HQ:"), 1, 0)
        conn_layout.addWidget(self._conn_ft2232hq, 1, 1)

        top_row.addWidget(conn_group)

        # Packet statistics
        stats_group = QGroupBox("Packet Statistics")
        stats_layout = QGridLayout(stats_group)

        labels = [
            "Radar Packets:", "Bytes Received:", "GPS Packets:",
            "Errors:", "Active Tracks:", "Detected Targets:",
            "Uptime:", "Packet Rate:",
        ]
        self._diag_values: list = []
        for i, text in enumerate(labels):
            r, c = divmod(i, 2)
            stats_layout.addWidget(QLabel(text), r, c * 2)
            val = QLabel("0")
            val.setStyleSheet(f"color: {DARK_INFO}; font-weight: bold;")
            stats_layout.addWidget(val, r, c * 2 + 1)
            self._diag_values.append(val)

        top_row.addWidget(stats_group)

        # Dependency status
        dep_group = QGroupBox("Optional Dependencies")
        dep_layout = QGridLayout(dep_group)

        deps = [
            ("pyusb", USB_AVAILABLE),
            ("pyftdi", FTDI_AVAILABLE),
            ("scipy", SCIPY_AVAILABLE),
            ("sklearn", SKLEARN_AVAILABLE),
            ("filterpy", FILTERPY_AVAILABLE),
            ("crcmod", CRCMOD_AVAILABLE),
        ]
        for i, (name, avail) in enumerate(deps):
            dep_layout.addWidget(QLabel(name), i, 0)
            lbl = QLabel("Available" if avail else "Missing")
            lbl.setStyleSheet(
                f"color: {DARK_SUCCESS}; font-weight: bold;"
                if avail else
                f"color: {DARK_WARNING}; font-weight: bold;"
            )
            dep_layout.addWidget(lbl, i, 1)

        top_row.addWidget(dep_group)

        layout.addLayout(top_row)

        # Log viewer
        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout(log_group)

        self._log_text = QPlainTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setMaximumBlockCount(500)
        log_layout.addWidget(self._log_text)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self._log_text.clear)
        log_layout.addWidget(clear_btn)

        layout.addWidget(log_group, stretch=1)

        self._tabs.addTab(tab, "Diagnostics")

    # -----------------------------------------------------------------
    # TAB 4: Settings
    # -----------------------------------------------------------------

    def _create_settings_tab(self):
        from PyQt6.QtWidgets import QScrollArea

        tab = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(8, 8, 8, 8)

        # ---- Radar parameters group ----------------------------------------
        radar_group = QGroupBox("Radar Parameters")
        r_layout = QGridLayout(radar_group)

        self._setting_spins: dict = {}
        param_defs = [
            ("System Frequency (GHz):", "system_frequency", 1, 100, 2,
             self._settings.system_frequency / 1e9, " GHz"),
            ("Chirp Duration 1 (us):", "chirp_duration_1", 0.01, 10000, 2,
             self._settings.chirp_duration_1 * 1e6, " us"),
            ("Chirp Duration 2 (us):", "chirp_duration_2", 0.001, 10000, 3,
             self._settings.chirp_duration_2 * 1e6, " us"),
            ("Chirps per Position:", "chirps_per_position", 1, 1024, 0,
             self._settings.chirps_per_position, ""),
            ("Freq Min (MHz):", "freq_min", 0.1, 1000, 1,
             self._settings.freq_min / 1e6, " MHz"),
            ("Freq Max (MHz):", "freq_max", 0.1, 1000, 1,
             self._settings.freq_max / 1e6, " MHz"),
            ("PRF 1 (Hz):", "prf1", 100, 100000, 0,
             self._settings.prf1, " Hz"),
            ("PRF 2 (Hz):", "prf2", 100, 100000, 0,
             self._settings.prf2, " Hz"),
            ("Max Distance (km):", "max_distance", 1, 500, 1,
             self._settings.max_distance / 1000, " km"),
            ("Map Size (km):", "map_size", 1, 500, 1,
             self._settings.map_size / 1000, " km"),
        ]

        for i, (label, key, lo, hi, dec, default, suffix) in enumerate(param_defs):
            r_layout.addWidget(QLabel(label), i, 0)
            if dec == 0:
                spin = QSpinBox()
                spin.setRange(int(lo), int(hi))
                spin.setValue(int(default))
                if suffix:
                    spin.setSuffix(suffix)
            else:
                spin = QDoubleSpinBox()
                spin.setRange(lo, hi)
                spin.setDecimals(dec)
                spin.setValue(default)
                if suffix:
                    spin.setSuffix(suffix)
            r_layout.addWidget(spin, i, 1)
            self._setting_spins[key] = spin

        apply_btn = QPushButton("Apply Settings")
        apply_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_INFO}; color: white; font-weight: bold; }}"
        )
        apply_btn.clicked.connect(self._apply_settings)
        r_layout.addWidget(apply_btn, len(param_defs), 0, 1, 2)

        layout.addWidget(radar_group)

        # ---- Signal Processing group ---------------------------------------
        proc_group = QGroupBox("Signal Processing")
        p_layout = QGridLayout(proc_group)
        row = 0

        # -- MTI --
        self._mti_check = QCheckBox("MTI (Moving Target Indication)")
        self._mti_check.setChecked(self._processing_config.mti_enabled)
        p_layout.addWidget(self._mti_check, row, 0, 1, 2)
        row += 1

        p_layout.addWidget(QLabel("MTI Order:"), row, 0)
        self._mti_order_spin = QSpinBox()
        self._mti_order_spin.setRange(1, 3)
        self._mti_order_spin.setValue(self._processing_config.mti_order)
        self._mti_order_spin.setToolTip("1 = single canceller, 2 = double, 3 = triple")
        p_layout.addWidget(self._mti_order_spin, row, 1)
        row += 1

        # -- Separator --
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.HLine)
        sep1.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep1, row, 0, 1, 2)
        row += 1

        # -- CFAR --
        self._cfar_check = QCheckBox("CFAR (Constant False Alarm Rate)")
        self._cfar_check.setChecked(self._processing_config.cfar_enabled)
        p_layout.addWidget(self._cfar_check, row, 0, 1, 2)
        row += 1

        p_layout.addWidget(QLabel("CFAR Type:"), row, 0)
        self._cfar_type_combo = QComboBox()
        self._cfar_type_combo.addItems(["CA-CFAR", "OS-CFAR", "GO-CFAR", "SO-CFAR"])
        self._cfar_type_combo.setCurrentText(self._processing_config.cfar_type)
        p_layout.addWidget(self._cfar_type_combo, row, 1)
        row += 1

        p_layout.addWidget(QLabel("Guard Cells:"), row, 0)
        self._cfar_guard_spin = QSpinBox()
        self._cfar_guard_spin.setRange(1, 20)
        self._cfar_guard_spin.setValue(self._processing_config.cfar_guard_cells)
        p_layout.addWidget(self._cfar_guard_spin, row, 1)
        row += 1

        p_layout.addWidget(QLabel("Training Cells:"), row, 0)
        self._cfar_train_spin = QSpinBox()
        self._cfar_train_spin.setRange(1, 50)
        self._cfar_train_spin.setValue(self._processing_config.cfar_training_cells)
        p_layout.addWidget(self._cfar_train_spin, row, 1)
        row += 1

        p_layout.addWidget(QLabel("Threshold Factor:"), row, 0)
        self._cfar_thresh_spin = QDoubleSpinBox()
        self._cfar_thresh_spin.setRange(0.1, 50.0)
        self._cfar_thresh_spin.setDecimals(1)
        self._cfar_thresh_spin.setValue(self._processing_config.cfar_threshold_factor)
        self._cfar_thresh_spin.setSingleStep(0.5)
        p_layout.addWidget(self._cfar_thresh_spin, row, 1)
        row += 1

        # -- Separator --
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep2, row, 0, 1, 2)
        row += 1

        # -- DC Notch --
        self._dc_notch_check = QCheckBox("DC Notch / Zero-Doppler Removal")
        self._dc_notch_check.setChecked(self._processing_config.dc_notch_enabled)
        p_layout.addWidget(self._dc_notch_check, row, 0, 1, 2)
        row += 1

        # -- Separator --
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.Shape.HLine)
        sep3.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep3, row, 0, 1, 2)
        row += 1

        # -- Windowing --
        p_layout.addWidget(QLabel("Window Function:"), row, 0)
        self._window_combo = QComboBox()
        self._window_combo.addItems(["None", "Hann", "Hamming", "Blackman", "Kaiser", "Chebyshev"])
        self._window_combo.setCurrentText(self._processing_config.window_type)
        if not SCIPY_AVAILABLE:
            # Without scipy, only None/Hann/Hamming/Blackman via numpy
            self._window_combo.setToolTip("Kaiser and Chebyshev require scipy")
        p_layout.addWidget(self._window_combo, row, 1)
        row += 1

        # -- Separator --
        sep4 = QFrame()
        sep4.setFrameShape(QFrame.Shape.HLine)
        sep4.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep4, row, 0, 1, 2)
        row += 1

        # -- Detection Threshold --
        p_layout.addWidget(QLabel("Detection Threshold (dB):"), row, 0)
        self._det_thresh_spin = QDoubleSpinBox()
        self._det_thresh_spin.setRange(0.0, 60.0)
        self._det_thresh_spin.setDecimals(1)
        self._det_thresh_spin.setValue(self._processing_config.detection_threshold_db)
        self._det_thresh_spin.setSuffix(" dB")
        self._det_thresh_spin.setSingleStep(1.0)
        self._det_thresh_spin.setToolTip("SNR threshold above noise floor (used when CFAR is disabled)")
        p_layout.addWidget(self._det_thresh_spin, row, 1)
        row += 1

        # -- Separator --
        sep5 = QFrame()
        sep5.setFrameShape(QFrame.Shape.HLine)
        sep5.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep5, row, 0, 1, 2)
        row += 1

        # -- Clustering --
        self._cluster_check = QCheckBox("DBSCAN Clustering")
        self._cluster_check.setChecked(self._processing_config.clustering_enabled)
        if not SKLEARN_AVAILABLE:
            self._cluster_check.setEnabled(False)
            self._cluster_check.setToolTip("Requires scikit-learn")
        p_layout.addWidget(self._cluster_check, row, 0, 1, 2)
        row += 1

        p_layout.addWidget(QLabel("DBSCAN eps:"), row, 0)
        self._cluster_eps_spin = QDoubleSpinBox()
        self._cluster_eps_spin.setRange(1.0, 5000.0)
        self._cluster_eps_spin.setDecimals(1)
        self._cluster_eps_spin.setValue(self._processing_config.clustering_eps)
        self._cluster_eps_spin.setSingleStep(10.0)
        p_layout.addWidget(self._cluster_eps_spin, row, 1)
        row += 1

        p_layout.addWidget(QLabel("Min Samples:"), row, 0)
        self._cluster_min_spin = QSpinBox()
        self._cluster_min_spin.setRange(1, 20)
        self._cluster_min_spin.setValue(self._processing_config.clustering_min_samples)
        p_layout.addWidget(self._cluster_min_spin, row, 1)
        row += 1

        # -- Separator --
        sep6 = QFrame()
        sep6.setFrameShape(QFrame.Shape.HLine)
        sep6.setStyleSheet(f"color: {DARK_BORDER};")
        p_layout.addWidget(sep6, row, 0, 1, 2)
        row += 1

        # -- Kalman Tracking --
        self._tracking_check = QCheckBox("Kalman Tracking")
        self._tracking_check.setChecked(self._processing_config.tracking_enabled)
        if not FILTERPY_AVAILABLE:
            self._tracking_check.setEnabled(False)
            self._tracking_check.setToolTip("Requires filterpy")
        p_layout.addWidget(self._tracking_check, row, 0, 1, 2)
        row += 1

        # Apply Processing button
        apply_proc_btn = QPushButton("Apply Processing Settings")
        apply_proc_btn.setStyleSheet(
            f"QPushButton {{ background-color: {DARK_SUCCESS}; color: white; font-weight: bold; }}"
            f"QPushButton:hover {{ background-color: #66BB6A; }}"
        )
        apply_proc_btn.clicked.connect(self._apply_processing_config)
        p_layout.addWidget(apply_proc_btn, row, 0, 1, 2)

        layout.addWidget(proc_group)

        # ---- About group ---------------------------------------------------
        about_group = QGroupBox("About")
        about_layout = QVBoxLayout(about_group)
        about_lbl = QLabel(
            "<b>PLFM Radar System GUI V7</b><br>"
            "PyQt6 Edition with Embedded Leaflet Map<br><br>"
            "<b>Data Interface:</b> FT2232HQ (USB 2.0)<br>"
            "<b>Map:</b> OpenStreetMap + Leaflet.js<br>"
            "<b>Framework:</b> PyQt6 + QWebEngine<br>"
            "<b>Version:</b> 7.0.0"
        )
        about_lbl.setStyleSheet(f"color: {DARK_TEXT}; padding: 12px;")
        about_layout.addWidget(about_lbl)

        layout.addWidget(about_group)
        layout.addStretch()

        scroll.setWidget(inner)
        tab_layout = QVBoxLayout(tab)
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.addWidget(scroll)

        self._tabs.addTab(tab, "Settings")

    # =====================================================================
    # Status bar
    # =====================================================================

    def _setup_statusbar(self):
        bar = QStatusBar()
        self.setStatusBar(bar)

        self._sb_status = QLabel("Ready")
        bar.addWidget(self._sb_status)

        self._sb_targets = QLabel("Targets: 0")
        bar.addPermanentWidget(self._sb_targets)

        self._sb_mode = QLabel("Idle")
        self._sb_mode.setStyleSheet(f"color: {DARK_INFO}; font-weight: bold;")
        bar.addPermanentWidget(self._sb_mode)

    # =====================================================================
    # Device management
    # =====================================================================

    def _refresh_devices(self):
        # STM32
        self._stm32_devices = self._stm32.list_devices()
        self._stm32_combo.clear()
        for d in self._stm32_devices:
            self._stm32_combo.addItem(d["description"])
        if self._stm32_devices:
            self._stm32_combo.setCurrentIndex(0)

        # FT2232HQ (primary)
        self._ft2232hq_devices = self._ft2232hq.list_devices()
        self._ft2232hq_combo.clear()
        for d in self._ft2232hq_devices:
            self._ft2232hq_combo.addItem(d["description"])
        if self._ft2232hq_devices:
            self._ft2232hq_combo.setCurrentIndex(0)

        logger.info(
            f"Devices refreshed: {len(self._stm32_devices)} STM32, "
            f"{len(self._ft2232hq_devices)} FT2232HQ"
        )

    # =====================================================================
    # Start / Stop radar
    # =====================================================================

    def _start_radar(self):
        try:
            # Open STM32
            idx = self._stm32_combo.currentIndex()
            if idx < 0 or idx >= len(self._stm32_devices):
                QMessageBox.warning(self, "Warning", "Please select an STM32 USB device.")
                return
            if not self._stm32.open_device(self._stm32_devices[idx]):
                QMessageBox.critical(self, "Error", "Failed to open STM32 USB device.")
                return

            # Open FT2232HQ (primary)
            idx2 = self._ft2232hq_combo.currentIndex()
            if idx2 >= 0 and idx2 < len(self._ft2232hq_devices):
                url = self._ft2232hq_devices[idx2]["url"]
                if not self._ft2232hq.open_device(url):
                    QMessageBox.warning(self, "Warning",
                                        "Failed to open FT2232HQ device. Radar data may not be available.")

            # Send start flag + settings
            if not self._stm32.send_start_flag():
                QMessageBox.critical(self, "Error", "Failed to send start flag to STM32.")
                return
            self._apply_settings_to_model()
            self._stm32.send_settings(self._settings)

            # Start workers
            self._radar_worker = RadarDataWorker(
                ft2232hq=self._ft2232hq,
                processor=self._processor,
                packet_parser=self._radar_parser,
                settings=self._settings,
                gps_data_ref=self._radar_position,
            )
            self._radar_worker.targetsUpdated.connect(self._on_radar_targets)
            self._radar_worker.statsUpdated.connect(self._on_radar_stats)
            self._radar_worker.errorOccurred.connect(self._on_worker_error)
            self._radar_worker.start()

            self._gps_worker = GPSDataWorker(
                stm32=self._stm32,
                usb_parser=self._usb_parser,
            )
            self._gps_worker.gpsReceived.connect(self._on_gps_received)
            self._gps_worker.errorOccurred.connect(self._on_worker_error)
            self._gps_worker.start()

            # UI state
            self._running = True
            self._start_time = time.time()
            self._start_btn.setEnabled(False)
            self._stop_btn.setEnabled(True)
            self._status_label_main.setText("Status: Radar running")
            self._sb_status.setText("Radar running")
            self._sb_mode.setText("Live")
            logger.info("Radar system started")

        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start radar: {e}")
            logger.error(f"Start radar error: {e}")

    def _stop_radar(self):
        self._running = False

        if self._radar_worker:
            self._radar_worker.stop()
            self._radar_worker.wait(2000)
            self._radar_worker = None

        if self._gps_worker:
            self._gps_worker.stop()
            self._gps_worker.wait(2000)
            self._gps_worker = None

        self._stm32.close()
        self._ft2232hq.close()

        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._status_label_main.setText("Status: Radar stopped")
        self._sb_status.setText("Radar stopped")
        self._sb_mode.setText("Idle")
        logger.info("Radar system stopped")

    # =====================================================================
    # Demo mode
    # =====================================================================

    def _start_demo(self):
        if self._simulator:
            return
        self._simulator = TargetSimulator(self._radar_position, self)
        self._simulator.targetsUpdated.connect(self._on_demo_targets)
        self._simulator.start(500)
        self._demo_mode = True
        self._sb_mode.setText("Demo Mode")
        self._sb_status.setText("Demo mode active")
        self._demo_btn_main.setText("Stop Demo")
        self._demo_btn_map.setText("Stop Demo")
        self._demo_btn_map.setChecked(True)
        logger.info("Demo mode started")

    def _stop_demo(self):
        if self._simulator:
            self._simulator.stop()
            self._simulator = None
        self._demo_mode = False
        self._sb_mode.setText("Idle" if not self._running else "Live")
        self._sb_status.setText("Demo stopped")
        self._demo_btn_main.setText("Start Demo")
        self._demo_btn_map.setText("Start Demo")
        self._demo_btn_map.setChecked(False)
        logger.info("Demo mode stopped")

    def _toggle_demo_main(self):
        if self._demo_mode:
            self._stop_demo()
        else:
            self._start_demo()

    def _toggle_demo_map(self, checked: bool):
        if checked:
            self._start_demo()
        else:
            self._stop_demo()

    def _add_demo_target(self):
        if self._simulator:
            self._simulator.add_random_target()
            logger.info("Added random demo target")

    # =====================================================================
    # Slots — data from workers / simulator
    # =====================================================================

    @pyqtSlot(list)
    def _on_radar_targets(self, targets: list):
        self._current_targets = targets
        self._map_widget.set_targets(targets)

    @pyqtSlot(dict)
    def _on_radar_stats(self, stats: dict):
        self._radar_stats = stats

    @pyqtSlot(str)
    def _on_worker_error(self, msg: str):
        logger.error(f"Worker error: {msg}")

    @pyqtSlot(object)
    def _on_gps_received(self, gps: GPSData):
        self._gps_packet_count += 1
        self._radar_position.latitude = gps.latitude
        self._radar_position.longitude = gps.longitude
        self._radar_position.altitude = gps.altitude
        self._radar_position.pitch = gps.pitch
        self._radar_position.timestamp = gps.timestamp

        self._map_widget.set_radar_position(self._radar_position)

        if self._simulator:
            self._simulator.set_radar_position(self._radar_position)

    @pyqtSlot(list)
    def _on_demo_targets(self, targets: list):
        self._current_targets = targets
        self._map_widget.set_targets(targets)
        self._sb_targets.setText(f"Targets: {len(targets)}")

    def _on_target_selected(self, target_id: int):
        for t in self._current_targets:
            if t.id == target_id:
                self._show_target_info(t)
                break

    def _show_target_info(self, target: RadarTarget):
        status = ("Approaching" if target.velocity > 1
                  else ("Receding" if target.velocity < -1 else "Stationary"))
        color = (DARK_ERROR if status == "Approaching"
                 else (DARK_INFO if status == "Receding" else DARK_TEXT))
        info = (
            f"<b>Target #{target.id}</b><br><br>"
            f"<b>Track ID:</b> {target.track_id}<br>"
            f"<b>Range:</b> {target.range:.1f} m<br>"
            f"<b>Velocity:</b> {target.velocity:+.1f} m/s<br>"
            f"<b>Azimuth:</b> {target.azimuth:.1f}\u00b0<br>"
            f"<b>Elevation:</b> {target.elevation:.1f}\u00b0<br>"
            f"<b>SNR:</b> {target.snr:.1f} dB<br>"
            f"<b>Class:</b> {target.classification}<br>"
            f'<b>Status:</b> <span style="color: {color}">{status}</span>'
        )
        self._target_info_label.setText(info)

    # =====================================================================
    # Position / coverage callbacks (map sidebar)
    # =====================================================================

    def _on_position_changed(self):
        self._radar_position.latitude = self._lat_spin.value()
        self._radar_position.longitude = self._lon_spin.value()
        self._radar_position.altitude = self._alt_spin.value()
        self._map_widget.set_radar_position(self._radar_position)
        if self._simulator:
            self._simulator.set_radar_position(self._radar_position)

    def _on_coverage_changed(self, value: float):
        radius_m = value * 1000
        self._settings.coverage_radius = radius_m
        self._map_widget.set_coverage_radius(radius_m)

    # =====================================================================
    # Settings
    # =====================================================================

    def _apply_settings_to_model(self):
        """Read spin values into the RadarSettings model."""
        s = self._settings
        sp = self._setting_spins
        s.system_frequency = sp["system_frequency"].value() * 1e9
        s.chirp_duration_1 = sp["chirp_duration_1"].value() * 1e-6
        s.chirp_duration_2 = sp["chirp_duration_2"].value() * 1e-6
        s.chirps_per_position = int(sp["chirps_per_position"].value())
        s.freq_min = sp["freq_min"].value() * 1e6
        s.freq_max = sp["freq_max"].value() * 1e6
        s.prf1 = sp["prf1"].value()
        s.prf2 = sp["prf2"].value()
        s.max_distance = sp["max_distance"].value() * 1000
        s.map_size = sp["map_size"].value() * 1000

    def _apply_settings(self):
        try:
            self._apply_settings_to_model()
            if self._stm32.is_open:
                self._stm32.send_settings(self._settings)
            logger.info("Radar settings applied")
            QMessageBox.information(self, "Settings", "Radar settings applied.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Invalid setting value: {e}")
            logger.error(f"Settings error: {e}")

    def _apply_processing_config(self):
        """Read signal processing controls into ProcessingConfig and push to processor."""
        try:
            cfg = ProcessingConfig(
                mti_enabled=self._mti_check.isChecked(),
                mti_order=self._mti_order_spin.value(),
                cfar_enabled=self._cfar_check.isChecked(),
                cfar_type=self._cfar_type_combo.currentText(),
                cfar_guard_cells=self._cfar_guard_spin.value(),
                cfar_training_cells=self._cfar_train_spin.value(),
                cfar_threshold_factor=self._cfar_thresh_spin.value(),
                dc_notch_enabled=self._dc_notch_check.isChecked(),
                window_type=self._window_combo.currentText(),
                detection_threshold_db=self._det_thresh_spin.value(),
                clustering_enabled=self._cluster_check.isChecked(),
                clustering_eps=self._cluster_eps_spin.value(),
                clustering_min_samples=self._cluster_min_spin.value(),
                tracking_enabled=self._tracking_check.isChecked(),
            )
            self._processing_config = cfg
            self._processor.set_config(cfg)
            logger.info(
                f"Processing config applied: MTI={cfg.mti_enabled}(order {cfg.mti_order}), "
                f"CFAR={cfg.cfar_enabled}({cfg.cfar_type}), DC_Notch={cfg.dc_notch_enabled}, "
                f"Window={cfg.window_type}, Threshold={cfg.detection_threshold_db} dB, "
                f"Clustering={cfg.clustering_enabled}, Tracking={cfg.tracking_enabled}"
            )
            QMessageBox.information(self, "Processing", "Signal processing settings applied.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to apply processing settings: {e}")
            logger.error(f"Processing config error: {e}")

    # =====================================================================
    # Periodic GUI refresh (100 ms timer)
    # =====================================================================

    def _refresh_gui(self):
        try:
            # GPS label
            gps = self._radar_position
            self._gps_label.setText(
                f"GPS: Lat {gps.latitude:.6f}, Lon {gps.longitude:.6f}, "
                f"Alt {gps.altitude:.1f}m"
            )

            # Pitch label with colour coding
            pitch_text = f"Pitch: {gps.pitch:+.1f}\u00b0"
            self._pitch_label.setText(pitch_text)
            if abs(gps.pitch) > 10:
                self._pitch_label.setStyleSheet(f"color: {DARK_ERROR}; font-weight: bold;")
            elif abs(gps.pitch) > 5:
                self._pitch_label.setStyleSheet(f"color: {DARK_WARNING}; font-weight: bold;")
            else:
                self._pitch_label.setStyleSheet(f"color: {DARK_SUCCESS}; font-weight: bold;")

            # Range-Doppler map
            self._rdm_canvas.update_map(self._processor.range_doppler_map)

            # Targets table (main tab)
            self._update_main_targets_table()

            # Status label (main tab)
            if self._running:
                pkt = self._radar_stats.get("packets", 0)
                self._status_label_main.setText(
                    f"Status: Running \u2014 Packets: {pkt} \u2014 Pitch: {gps.pitch:+.1f}\u00b0"
                )

            # Diagnostics values
            self._update_diagnostics()

            # Status-bar target count
            self._sb_targets.setText(f"Targets: {len(self._current_targets)}")

        except Exception as e:
            logger.error(f"GUI refresh error: {e}")

    def _update_main_targets_table(self):
        targets = self._current_targets[-20:]  # last 20
        self._targets_table_main.setRowCount(len(targets))

        for row, t in enumerate(targets):
            self._targets_table_main.setItem(
                row, 0, QTableWidgetItem(str(t.track_id)))
            self._targets_table_main.setItem(
                row, 1, QTableWidgetItem(f"{t.range:.1f}"))

            vel_item = QTableWidgetItem(f"{t.velocity:+.1f}")
            if t.velocity > 1:
                vel_item.setForeground(QColor(DARK_ERROR))
            elif t.velocity < -1:
                vel_item.setForeground(QColor(DARK_INFO))
            self._targets_table_main.setItem(row, 2, vel_item)

            self._targets_table_main.setItem(
                row, 3, QTableWidgetItem(f"{t.azimuth:.1f}"))

            # Raw elevation — show stored value from corrections cache
            raw_text = "N/A"
            for corr in self._corrected_elevations[-20:]:
                if abs(corr["corrected"] - t.elevation) < 0.1:
                    raw_text = f"{corr['raw']}"
                    break
            self._targets_table_main.setItem(
                row, 4, QTableWidgetItem(raw_text))
            self._targets_table_main.setItem(
                row, 5, QTableWidgetItem(f"{t.elevation:.1f}"))
            self._targets_table_main.setItem(
                row, 6, QTableWidgetItem(f"{t.snr:.1f}"))

    def _update_diagnostics(self):
        # Connection indicators
        self._set_conn_indicator(self._conn_stm32, self._stm32.is_open)
        self._set_conn_indicator(self._conn_ft2232hq, self._ft2232hq.is_open)

        stats = self._radar_stats
        gps_count = self._gps_packet_count
        if self._gps_worker:
            gps_count = self._gps_worker.gps_count

        uptime = time.time() - self._start_time
        pkt = stats.get("packets", 0)
        pkt_rate = pkt / max(uptime, 1)

        vals = [
            str(pkt),
            f"{stats.get('bytes', 0):,}",
            str(gps_count),
            str(stats.get("errors", 0)),
            str(stats.get("active_tracks", len(self._processor.tracks))),
            str(stats.get("targets", len(self._current_targets))),
            f"{uptime:.0f}s",
            f"{pkt_rate:.1f}/s",
        ]
        for lbl, v in zip(self._diag_values, vals):
            lbl.setText(v)

    # =====================================================================
    # Helpers
    # =====================================================================

    @staticmethod
    def _make_status_label(name: str) -> QLabel:
        lbl = QLabel("Disconnected")
        lbl.setStyleSheet(f"color: {DARK_ERROR}; font-weight: bold;")
        return lbl

    @staticmethod
    def _set_conn_indicator(label: QLabel, connected: bool):
        if connected:
            label.setText("Connected")
            label.setStyleSheet(f"color: {DARK_SUCCESS}; font-weight: bold;")
        else:
            label.setText("Disconnected")
            label.setStyleSheet(f"color: {DARK_ERROR}; font-weight: bold;")

    def _log_append(self, message: str):
        """Append a log message to the diagnostics log viewer."""
        self._log_text.appendPlainText(message)

    # =====================================================================
    # Close event
    # =====================================================================

    def closeEvent(self, event):
        if self._simulator:
            self._simulator.stop()
        if self._radar_worker:
            self._radar_worker.stop()
            self._radar_worker.wait(1000)
        if self._gps_worker:
            self._gps_worker.stop()
            self._gps_worker.wait(1000)
        self._stm32.close()
        self._ft2232hq.close()
        logging.getLogger().removeHandler(self._log_handler)
        event.accept()


# =============================================================================
# Qt-compatible log handler (routes Python logging → QTextEdit)
# =============================================================================

class _QtLogHandler(logging.Handler):
    """Sends log records to a callback (called on the thread that emitted)."""

    def __init__(self, callback):
        super().__init__()
        self._callback = callback
        self.setFormatter(logging.Formatter(
            "%(asctime)s  %(levelname)-8s  %(message)s",
            datefmt="%H:%M:%S",
        ))

    def emit(self, record):
        try:
            msg = self.format(record)
            self._callback(msg)
        except Exception:
            pass
