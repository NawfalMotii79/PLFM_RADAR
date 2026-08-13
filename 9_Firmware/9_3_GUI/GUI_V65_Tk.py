#!/usr/bin/env python3
"""
AERIS-10 Radar Dashboard (Tkinter)
===================================================
Real-time visualization and control for the AERIS-10 phased-array radar
via FT2232H USB 2.0 interface.

Features:
  - FT2232H USB reader with packet parsing (matches usb_data_interface_ft2232h.v)
  - Real-time range-Doppler magnitude heatmap (64x32)
  - CFAR detection overlay (flagged cells highlighted)
  - Range profile waterfall plot (range vs. time)
  - North-up PPI raster (demo, phosphor decay)
  - Host command sender (opcodes per radar_system_top.v:
    0x01-0x04, 0x10-0x16, 0x20-0x27, 0x30-0x31, 0xFF)
  - Configuration panel for all radar parameters
  - HDF5 data recording for offline analysis
  - Replay mode (co-sim dirs, raw IQ .npy, HDF5) with transport controls
  - Demo mode with synthetic moving targets
  - Detected targets table
  - Dual dispatch: FPGA controls route to SoftwareFPGA during replay
  - Mock mode for development/testing without hardware

Usage:
  python GUI_V65_Tk.py              # Launch with mock data
  python GUI_V65_Tk.py --live       # Launch with FT2232H hardware
  python GUI_V65_Tk.py --record     # Launch with HDF5 recording
  python GUI_V65_Tk.py --replay path/to/data  # Auto-load replay
  python GUI_V65_Tk.py --demo       # Start in demo mode (100 PPI targets)
"""

import os
import time
import copy
import queue
import random
import logging
import argparse
import threading
import contextlib
from collections import deque
from pathlib import Path
from typing import ClassVar

import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk, filedialog

    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.patches import Circle
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    _HAS_GUI = True
except (ModuleNotFoundError, ImportError):
    _HAS_GUI = False

# Import protocol layer (no GUI deps)
from radar_protocol import (
    RadarProtocol, FT2232HConnection, FT601Connection,
    DataRecorder, RadarAcquisition,
    RadarFrame, StatusResponse,
    NUM_RANGE_BINS, NUM_DOPPLER_BINS, WATERFALL_DEPTH,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("GUI_V65_Tk")



# ============================================================================
# Dashboard GUI
# ============================================================================

# Dark theme colors
BG = "#1e1e2e"
BG2 = "#282840"
FG = "#cdd6f4"
ACCENT = "#89b4fa"
GREEN = "#a6e3a1"
RED = "#f38ba8"
YELLOW = "#f9e2af"
SURFACE = "#313244"


PPI_SIZE = 256
PPI_DECAY = 0.93
PPI_SWEEP_RPS = 0.4
PPI_BEAM_DEG = 10.0
PPI_BG = "#000000"
PPI_PHOS = "#3dff7a"
PPI_PHOS_DIM = "#145c2e"
DEMO_N_DEFAULT = 100
DEMO_N_MAX = 100
DEMO_INTERVAL_MS = 50
_TABLE_K = 20
_CLASS_NAMES = ("aircraft", "drone", "bird", "unknown")


def ppi_xy(range_m, azimuth_deg) -> np.ndarray:
    """North-up PPI coordinates in metres. 0°=North, 90°=East (clockwise).

    Returns (N, 2) as (east, north). Empty input → shape (0, 2).
    """
    r = np.atleast_1d(np.asarray(range_m, dtype=np.float64))
    az = np.radians(np.atleast_1d(np.asarray(azimuth_deg, dtype=np.float64)))
    if r.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return np.column_stack((r * np.sin(az), r * np.cos(az)))


def ppi_raster(xy, max_range: float, *, size: int = PPI_SIZE,
               decay: float = 0.0, out: np.ndarray | None = None,
               weights=None) -> np.ndarray:
    """Stamp PPI samples into a dense image. O(N) index writes, no artists."""
    if out is None:
        out = np.zeros((size, size), dtype=np.float32)
    if decay:
        out *= np.float32(decay)
    else:
        out.fill(0.0)
    if xy is None or xy.size == 0 or max_range <= 0:
        return out
    s = (size - 1) * 0.5
    scale = s / max_range
    cols = np.rint(xy[:, 0] * scale + s).astype(np.intp)
    rows = np.rint(xy[:, 1] * scale + s).astype(np.intp)
    m = (cols >= 0) & (cols < size) & (rows >= 0) & (rows < size)
    if not np.any(m):
        return out
    if weights is None:
        out[rows[m], cols[m]] = 1.0
    else:
        w = np.asarray(weights, dtype=np.float32)[m]
        out[rows[m], cols[m]] = np.maximum(out[rows[m], cols[m]], w)
    return out


def ppi_beam_weights(azimuth_deg, sweep_az: float, beam_deg: float = PPI_BEAM_DEG):
    """Gaussian illumination vs sweep heading. Peak 1 at the beam."""
    d = (np.asarray(azimuth_deg, dtype=np.float64) - sweep_az + 180.0) % 360.0 - 180.0
    sigma = max(beam_deg, 1e-3)
    return np.exp(-0.5 * (d / sigma) ** 2).astype(np.float32)


# ============================================================================
# Demo Target Simulator (Tkinter timer-based)
# ============================================================================

class DemoTarget:
    """Single simulated target with kinematics."""

    __slots__ = ("azimuth", "classification", "id", "range_m", "snr", "velocity")

    # Physical range grid: 64 bins x ~24 m/bin = ~1536 m max
    # Bin spacing = c / (2 * Fs) * decimation, where Fs = 100 MHz DDC output.
    _RANGE_PER_BIN: float = (3e8 / (2 * 100e6)) * 16  # ~24 m
    _MAX_RANGE: float = _RANGE_PER_BIN * NUM_RANGE_BINS  # ~1536 m

    def __init__(self, tid: int):
        self.id = tid
        self.range_m = random.uniform(20, self._MAX_RANGE - 20)
        self.velocity = random.uniform(-10, 10)
        self.azimuth = random.uniform(0, 360)
        self.snr = random.uniform(10, 35)
        self.classification = random.choice(
            ["aircraft", "drone", "bird", "unknown"])

    def step(self) -> bool:
        """Advance one tick.  Return False if target exits coverage."""
        self.range_m -= self.velocity * 0.1
        if self.range_m < 5 or self.range_m > self._MAX_RANGE:
            return False
        self.velocity = max(-20, min(20, self.velocity + random.uniform(-1, 1)))
        self.azimuth = (self.azimuth + random.uniform(-0.5, 0.5)) % 360
        self.snr = max(0, min(50, self.snr + random.uniform(-1, 1)))
        return True


class DemoSimulator:
    """Vectorized demo swarm. Kinematics stay in numpy; UI gets arrays + top-K rows."""

    _RANGE_PER_BIN: float = DemoTarget._RANGE_PER_BIN
    _MAX_RANGE: float = DemoTarget._MAX_RANGE
    _VEL_PER_BIN: float = 5.34

    def __init__(self, frame_queue: queue.Queue, ui_queue: queue.Queue,
                 root: tk.Tk, interval_ms: int = 500, n_targets: int = 8,
                 on_ppi=None):
        self._frame_queue = frame_queue
        self._ui_queue = ui_queue
        self._root = root
        self._interval_ms = interval_ms
        self._on_ppi = on_ppi
        self._frame_number = 0
        self._after_id: str | None = None
        self._tick_i = 0
        self._rng = np.random.default_rng()
        self.n_targets = max(1, int(n_targets))
        self._alloc(self.n_targets)

    def start(self):
        self._tick()

    def stop(self):
        if self._after_id is not None:
            self._root.after_cancel(self._after_id)
            self._after_id = None

    def add_random_target(self):
        """Keep API: replace the weakest-SNR slot instead of appending."""
        i = int(np.argmin(self._snr))
        mask = np.zeros(self.n_targets, dtype=bool)
        mask[i] = True
        self._spawn_into(mask)

    def _alloc(self, n: int):
        rng = self._rng
        self._id = np.arange(1, n + 1, dtype=np.int32)
        self._range = rng.uniform(20.0, self._MAX_RANGE - 20.0, n)
        self._vel = rng.uniform(-10.0, 10.0, n)
        self._az = rng.uniform(0.0, 360.0, n)
        self._snr = rng.uniform(10.0, 35.0, n)
        self._cls = rng.integers(0, 4, n)
        self._next_id = n + 1

    def _spawn_into(self, mask: np.ndarray):
        k = int(mask.sum())
        if k == 0:
            return
        rng = self._rng
        self._range[mask] = rng.uniform(20.0, self._MAX_RANGE - 20.0, k)
        self._vel[mask] = rng.uniform(-10.0, 10.0, k)
        self._az[mask] = rng.uniform(0.0, 360.0, k)
        self._snr[mask] = rng.uniform(10.0, 35.0, k)
        self._cls[mask] = rng.integers(0, 4, k)
        self._id[mask] = np.arange(self._next_id, self._next_id + k, dtype=np.int32)
        self._next_id += k

    def _step(self):
        scale = self._interval_ms / 500.0
        n = self.n_targets
        rng = self._rng
        self._range -= self._vel * (0.1 * scale)
        self._vel = np.clip(self._vel + rng.uniform(-1.0, 1.0, n) * scale, -20.0, 20.0)
        self._az = (self._az + rng.uniform(-0.5, 0.5, n) * scale) % 360.0
        self._snr = np.clip(self._snr + rng.uniform(-1.0, 1.0, n) * scale, 0.0, 50.0)
        self._spawn_into((self._range < 5.0) | (self._range > self._MAX_RANGE))

    def _table_rows(self) -> list[dict]:
        k = min(_TABLE_K, self.n_targets)
        idx = np.argpartition(self._snr, -k)[-k:]
        idx = idx[np.argsort(self._snr[idx])[::-1]]
        return [
            {"id": int(self._id[i]), "range_m": float(self._range[i]),
             "velocity": float(self._vel[i]), "azimuth": float(self._az[i]),
             "snr": float(self._snr[i]), "class": _CLASS_NAMES[int(self._cls[i])]}
            for i in idx
        ]

    def _tick(self):
        self._step()
        with contextlib.suppress(queue.Full):
            self._frame_queue.put_nowait(self._make_frame())
        if self._on_ppi is not None:
            self._on_ppi(self._range, self._az)
        self._tick_i += 1
        if self._tick_i % 10 == 1:
            self._ui_queue.put(("demo_targets", self._table_rows()))
        self._after_id = self._root.after(self._interval_ms, self._tick)

    def _make_frame(self) -> RadarFrame:
        mag = np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.float64)
        r_bin = np.clip(
            (self._range / self._RANGE_PER_BIN).astype(np.intp), 0, NUM_RANGE_BINS - 1)
        d_bin = np.clip(
            np.rint(self._vel / self._VEL_PER_BIN + NUM_DOPPLER_BINS / 2).astype(np.intp),
            0, NUM_DOPPLER_BINS - 1)
        np.add.at(mag, (r_bin, d_bin), 500.0 + self._snr * 200.0)
        det = (mag > 0).astype(np.uint8)
        self._frame_number += 1
        return RadarFrame(
            timestamp=time.time(),
            range_doppler_i=(mag * 0.5).astype(np.int16),
            range_doppler_q=np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS), dtype=np.int16),
            magnitude=mag,
            detections=det,
            range_profile=mag.max(axis=1),
            detection_count=int(det.sum()),
            frame_number=self._frame_number,
        )


# ============================================================================
# Replay Controller (threading-based, reuses v7.ReplayEngine)
# ============================================================================

class _ReplayController:
    """Manages replay playback in a background thread for the Tkinter dashboard.

    Imports ``ReplayEngine`` and ``SoftwareFPGA`` from ``v7`` lazily so
    they are only required when replay is actually used.
    """

    # Speed multiplier → frame interval in seconds
    SPEED_MAP: ClassVar[dict[str, float]] = {
        "0.25x": 0.400,
        "0.5x": 0.200,
        "1x": 0.100,
        "2x": 0.050,
        "5x": 0.020,
        "10x": 0.010,
    }

    def __init__(self, frame_queue: queue.Queue, ui_queue: queue.Queue):
        self._frame_queue = frame_queue
        self._ui_queue = ui_queue
        self._engine = None  # lazy
        self._software_fpga = None  # lazy
        self._thread: threading.Thread | None = None
        self._play_event = threading.Event()
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._current_index = 0
        self._last_emitted_index = -1
        self._loop = False
        self._frame_interval = 0.100  # 1x speed

    def load(self, path: str) -> int:
        """Load replay data from path.  Returns total frames or raises."""
        from v7.replay import ReplayEngine, ReplayFormat, detect_format
        from v7.software_fpga import SoftwareFPGA

        fmt = detect_format(path)
        if fmt == ReplayFormat.RAW_IQ_NPY:
            self._software_fpga = SoftwareFPGA()
            self._engine = ReplayEngine(path, software_fpga=self._software_fpga)
        else:
            self._engine = ReplayEngine(path)

        self._current_index = 0
        self._last_emitted_index = -1
        self._stop_event.clear()
        self._play_event.clear()
        return self._engine.total_frames

    @property
    def total_frames(self) -> int:
        return self._engine.total_frames if self._engine else 0

    @property
    def current_index(self) -> int:
        return self._last_emitted_index if self._last_emitted_index >= 0 else 0

    @property
    def is_playing(self) -> bool:
        return self._play_event.is_set()

    @property
    def software_fpga(self):
        return self._software_fpga

    def set_speed(self, label: str):
        self._frame_interval = self.SPEED_MAP.get(label, 0.100)

    def set_loop(self, loop: bool):
        self._loop = loop

    def play(self):
        self._play_event.set()
        with self._lock:
            if self._current_index >= self.total_frames:
                self._current_index = 0
        self._ui_queue.put(("replay_state", "playing"))
        if self._thread is None or not self._thread.is_alive():
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def pause(self):
        self._play_event.clear()
        self._ui_queue.put(("replay_state", "paused"))

    def stop(self):
        self._stop_event.set()
        self._play_event.set()  # unblock wait so thread exits promptly
        with self._lock:
            self._current_index = 0
            self._last_emitted_index = -1
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        self._play_event.clear()
        self._ui_queue.put(("replay_state", "stopped"))

    def close(self):
        """Stop playback and release underlying engine resources."""
        self.stop()
        if self._engine is not None:
            self._engine.close()
            self._engine = None
        self._software_fpga = None

    def seek(self, index: int):
        with self._lock:
            self._current_index = max(0, min(index, self.total_frames - 1))
            self._emit_frame()
            self._last_emitted_index = self._current_index
            # Advance past the emitted frame so _run doesn't re-emit it
            self._current_index += 1

    def _run(self):
        while not self._stop_event.is_set():
            # Block until play or stop is signalled — no busy-sleep
            self._play_event.wait()
            if self._stop_event.is_set():
                break
            with self._lock:
                if self._current_index >= self.total_frames:
                    if self._loop:
                        self._current_index = 0
                    else:
                        self._play_event.clear()
                        self._ui_queue.put(("replay_state", "paused"))
                        continue
                self._emit_frame()
                self._last_emitted_index = self._current_index
                idx = self._current_index
                self._current_index += 1
            self._ui_queue.put(("replay_index", (idx, self.total_frames)))
            time.sleep(self._frame_interval)

    def _emit_frame(self):
        """Get current frame and push to queue. Must be called with lock held."""
        if self._engine is None:
            return
        frame = self._engine.get_frame(self._current_index)
        if frame is not None:
            frame = copy.deepcopy(frame)
            with contextlib.suppress(queue.Full):
                self._frame_queue.put_nowait(frame)


class RadarDashboard:
    """Main tkinter application: real-time radar visualization and control."""

    UPDATE_INTERVAL_MS = 100  # 10 Hz display refresh

    # Radar parameters used for range-axis scaling.
    SAMPLE_RATE = 100e6      # Hz — DDC output I/Q rate (matched filter input)
    C = 3e8                  # m/s — speed of light

    def __init__(self, root: tk.Tk, mock: bool,
                 recorder: DataRecorder, device_index: int = 0):
        self.root = root
        self._mock = mock
        self.conn: FT2232HConnection | FT601Connection | None = None
        self.recorder = recorder
        self.device_index = device_index

        self.root.title("AERIS-10 Radar Dashboard")
        self.root.geometry("1600x950")
        self.root.configure(bg=BG)

        # Frame queue (acquisition / replay / demo → display)
        self.frame_queue: queue.Queue[RadarFrame] = queue.Queue(maxsize=8)
        self._acq_thread: RadarAcquisition | None = None

        # Thread-safe UI message queue — avoids calling root.after() from
        # background threads which crashes Python 3.12 (GIL state corruption).
        # Entries are (tag, payload) tuples drained by _schedule_update().
        self._ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()

        # Display state
        self._current_frame = RadarFrame()
        self._waterfall = deque(maxlen=WATERFALL_DEPTH)
        for _ in range(WATERFALL_DEPTH):
            self._waterfall.append(np.zeros(NUM_RANGE_BINS))

        self._frame_count = 0
        self._fps_ts = time.time()
        self._fps = 0.0

        # Stable colorscale — exponential moving average of vmax
        self._vmax_ema = 1000.0
        self._vmax_alpha = 0.15  # smoothing factor (lower = more stable)

        # AGC visualization history (ring buffers, ~60s at 10 Hz)
        self._agc_history_len = 256
        self._agc_gain_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_peak_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_sat_history: deque[int] = deque(maxlen=self._agc_history_len)
        self._agc_time_history: deque[float] = deque(maxlen=self._agc_history_len)
        self._agc_t0: float = time.time()
        self._agc_last_redraw: float = 0.0  # throttle chart redraws
        self._AGC_REDRAW_INTERVAL: float = 0.5  # seconds between redraws

        # Replay state
        self._replay_ctrl: _ReplayController | None = None
        self._replay_active = False

        # Demo state
        self._demo_sim: DemoSimulator | None = None
        self._demo_active = False
        self._demo_n = DEMO_N_DEFAULT

        # Detected targets (from demo or replay host-DSP)
        self._detected_targets: list[dict] = []

        self._build_ui()
        self._schedule_update()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG, fieldbackground=SURFACE)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("TButton", background=SURFACE, foreground=FG)
        style.configure("TLabelframe", background=BG, foreground=ACCENT)
        style.configure("TLabelframe.Label", background=BG, foreground=ACCENT)
        style.configure("Accent.TButton", background=ACCENT, foreground=BG)
        style.configure("TNotebook", background=BG)
        style.configure("TNotebook.Tab", background=SURFACE, foreground=FG,
                         padding=[12, 4])
        style.map("TNotebook.Tab", background=[("selected", ACCENT)],
                  foreground=[("selected", BG)])

        # Top bar
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=(8, 0))

        self.lbl_status = ttk.Label(top, text="DISCONNECTED", foreground=RED,
                                     font=("Menlo", 11, "bold"))
        self.lbl_status.pack(side="left", padx=8)

        self.lbl_fps = ttk.Label(top, text="0.0 fps", font=("Menlo", 10))
        self.lbl_fps.pack(side="left", padx=16)

        self.lbl_detections = ttk.Label(top, text="Det: 0", font=("Menlo", 10))
        self.lbl_detections.pack(side="left", padx=16)

        self.lbl_frame = ttk.Label(top, text="Frame: 0", font=("Menlo", 10))
        self.lbl_frame.pack(side="left", padx=16)

        self.btn_connect = ttk.Button(top, text="Connect",
                                       command=self._on_connect,
                                       style="Accent.TButton")
        self.btn_connect.pack(side="right", padx=4)

        # USB Interface selector (production FT2232H / premium FT601)
        self._usb_iface_var = tk.StringVar(value="FT2232H (Production)")
        self.cmb_usb_iface = ttk.Combobox(
            top, textvariable=self._usb_iface_var,
            values=["FT2232H (Production)", "FT601 (Premium)"],
            state="readonly", width=20,
        )
        self.cmb_usb_iface.pack(side="right", padx=4)
        ttk.Label(top, text="USB:", font=("Menlo", 10)).pack(side="right")

        self.btn_record = ttk.Button(top, text="Record", command=self._on_record)
        self.btn_record.pack(side="right", padx=4)

        self.btn_demo = ttk.Button(top, text="Start Demo",
                                    command=self._toggle_demo)
        self.btn_demo.pack(side="right", padx=4)

        # -- Tabbed notebook layout --
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        tab_display = ttk.Frame(nb)
        tab_control = ttk.Frame(nb)
        tab_replay = ttk.Frame(nb)
        tab_agc = ttk.Frame(nb)
        tab_log = ttk.Frame(nb)
        nb.add(tab_display, text="  Display  ")
        nb.add(tab_control, text="  Control  ")
        nb.add(tab_replay, text="  Replay  ")
        nb.add(tab_agc, text="  AGC Monitor  ")
        nb.add(tab_log, text="  Log  ")

        self._build_display_tab(tab_display)
        self._build_control_tab(tab_control)
        self._build_replay_tab(tab_replay)
        self._build_agc_tab(tab_agc)
        self._build_log_tab(tab_log)

    def _build_display_tab(self, parent):
        # Compute physical axis limits
        # Bin spacing = c / (2 * Fs_ddc) for matched-filter processing.
        range_per_bin = self.C / (2.0 * self.SAMPLE_RATE) * 16  # ~24 m
        max_range = range_per_bin * NUM_RANGE_BINS

        doppler_bin_lo = 0
        doppler_bin_hi = NUM_DOPPLER_BINS

        # Top pane: plots
        plot_frame = ttk.Frame(parent)
        plot_frame.pack(fill="both", expand=True)

        # One figure so PPI cannot be packed off-screen. Raster blit stays O(pixels).
        self.fig = Figure(figsize=(16, 5), facecolor=BG)
        self.fig.subplots_adjust(left=0.05, right=0.99, top=0.94, bottom=0.10,
                                  wspace=0.28, hspace=0.35)

        # Range-Doppler heatmap
        self.ax_rd = self.fig.add_subplot(1, 4, (1, 2))
        self.ax_rd.set_facecolor(BG2)
        self._rd_img = self.ax_rd.imshow(
            np.zeros((NUM_RANGE_BINS, NUM_DOPPLER_BINS)),
            aspect="auto", cmap="inferno", origin="lower",
            extent=[doppler_bin_lo, doppler_bin_hi, 0, max_range],
            vmin=0, vmax=1000,
        )
        self.ax_rd.set_title("Range-Doppler Map", color=FG, fontsize=12)
        self.ax_rd.set_xlabel("Doppler Bin (0-15: long PRI, 16-31: short PRI)", color=FG)
        self.ax_rd.set_ylabel("Range (m)", color=FG)
        self.ax_rd.tick_params(colors=FG)

        # Save axis limits for coordinate conversions
        self._max_range = max_range
        self._range_per_bin = range_per_bin

        # CFAR detection overlay (scatter)
        self._det_scatter = self.ax_rd.scatter([], [], s=30, c=GREEN,
                                                marker="x", linewidths=1.5,
                                                zorder=5, label="CFAR Det")

        # Waterfall plot (range profile vs time)
        self.ax_wf = self.fig.add_subplot(1, 4, 3)
        self.ax_wf.set_facecolor(BG2)
        wf_init = np.zeros((WATERFALL_DEPTH, NUM_RANGE_BINS))
        self._wf_img = self.ax_wf.imshow(
            wf_init, aspect="auto", cmap="viridis", origin="lower",
            extent=[0, max_range, 0, WATERFALL_DEPTH],
            vmin=0, vmax=5000,
        )
        self.ax_wf.set_title("Range Waterfall", color=FG, fontsize=12)
        self.ax_wf.set_xlabel("Range (m)", color=FG)
        self.ax_wf.set_ylabel("Frame", color=FG)
        self.ax_wf.tick_params(colors=FG)

        # Classic circular PPI: phosphor raster, range rings, spokes, rotating sweep.
        self._ppi_buf = np.zeros((PPI_SIZE, PPI_SIZE), dtype=np.float32)
        self.ax_ppi = self.fig.add_subplot(1, 4, 4)
        self.ax_ppi.set_facecolor(PPI_BG)
        self.ax_ppi.set_aspect("equal")
        phosphor = LinearSegmentedColormap.from_list(
            "phosphor", [PPI_BG, PPI_PHOS_DIM, PPI_PHOS, "#e8ffe8"])
        self._ppi_img = self.ax_ppi.imshow(
            self._ppi_buf, origin="lower", cmap=phosphor, interpolation="nearest",
            extent=[-max_range, max_range, -max_range, max_range],
            vmin=0.0, vmax=1.0, zorder=1)
        clip = Circle((0.0, 0.0), max_range, transform=self.ax_ppi.transData)
        self._ppi_img.set_clip_path(clip)
        self.ax_ppi.set_title("PPI", color=PPI_PHOS, fontsize=12)
        self.ax_ppi.set_xticks([])
        self.ax_ppi.set_yticks([])
        self.ax_ppi.tick_params(length=0)
        for spine in self.ax_ppi.spines.values():
            spine.set_visible(False)
        pad = max_range * 1.18
        self.ax_ppi.set_xlim(-pad, pad)
        self.ax_ppi.set_ylim(-pad, pad)
        ring_kw = {"fill": False, "ec": PPI_PHOS, "lw": 0.7, "alpha": 0.35, "zorder": 2}
        for frac in (0.25, 0.5, 0.75, 1.0):
            self.ax_ppi.add_patch(Circle((0.0, 0.0), max_range * frac, **ring_kw))
        spoke_r = np.array([0.0, max_range])
        for deg in range(0, 360, 30):
            rad = np.radians(deg)
            self.ax_ppi.plot(
                spoke_r * np.sin(rad), spoke_r * np.cos(rad),
                color=PPI_PHOS, lw=0.4, alpha=0.28, zorder=2, solid_capstyle="round")
        for label, deg in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            rad = np.radians(deg)
            rr = max_range * 1.08
            self.ax_ppi.text(
                rr * np.sin(rad), rr * np.cos(rad), label,
                color=PPI_PHOS, fontsize=8, ha="center", va="center", zorder=4)
        self._sweep_line, = self.ax_ppi.plot(
            [0.0, 0.0], [0.0, max_range],
            color=PPI_PHOS, lw=1.4, alpha=0.85, zorder=3)

        canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        canvas.draw()
        self._ppi_img.set_clip_path(
            Circle((0.0, 0.0), max_range, transform=self.ax_ppi.transData))
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

        # Bottom pane: detected targets table
        tgt_frame = ttk.LabelFrame(parent, text="Detected Targets", padding=4)
        tgt_frame.pack(fill="x", padx=8, pady=(0, 4))

        cols = ("id", "range_m", "velocity", "azimuth", "snr", "class")
        self._tgt_tree = ttk.Treeview(
            tgt_frame, columns=cols, show="headings", height=5)
        for col, heading, width in [
            ("id", "ID", 50),
            ("range_m", "Range (m)", 100),
            ("velocity", "Vel (m/s)", 90),
            ("azimuth", "Az (deg)", 90),
            ("snr", "SNR (dB)", 80),
            ("class", "Class", 100),
        ]:
            self._tgt_tree.heading(col, text=heading)
            self._tgt_tree.column(col, width=width, anchor="center")

        scrollbar = ttk.Scrollbar(
            tgt_frame, orient="vertical", command=self._tgt_tree.yview)
        self._tgt_tree.configure(yscrollcommand=scrollbar.set)
        self._tgt_tree.pack(side="left", fill="x", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _build_control_tab(self, parent):
        """Host command sender — organized by FPGA register groups.

        Layout: scrollable canvas with three columns:
          Left:   Quick Actions + Diagnostics (self-test)
          Center: Waveform Timing + Signal Processing
          Right:  Detection (CFAR) + Custom Command
        """
        # Scrollable wrapper for small screens
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        outer = ttk.Frame(canvas)
        outer.bind("<Configure>",
                   lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=outer, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True, padx=8, pady=8)
        scrollbar.pack(side="right", fill="y")

        self._param_vars: dict[str, tk.StringVar] = {}

        # ── Left column: Quick Actions + Diagnostics ──────────────────
        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        # -- Radar Operation --
        grp_op = ttk.LabelFrame(left, text="Radar Operation", padding=10)
        grp_op.pack(fill="x", pady=(0, 8))

        ttk.Button(grp_op, text="Radar Mode On",
                   command=lambda: self._send_cmd(0x01, 1)).pack(fill="x", pady=2)
        ttk.Button(grp_op, text="Radar Mode Off",
                   command=lambda: self._send_cmd(0x01, 0)).pack(fill="x", pady=2)
        ttk.Button(grp_op, text="Trigger Chirp",
                   command=lambda: self._send_cmd(0x02, 1)).pack(fill="x", pady=2)

        # Stream Control (3-bit mask)
        sc_row = ttk.Frame(grp_op)
        sc_row.pack(fill="x", pady=2)
        ttk.Label(sc_row, text="Stream Control").pack(side="left")
        var_sc = tk.StringVar(value="7")
        self._param_vars["4"] = var_sc
        ttk.Entry(sc_row, textvariable=var_sc, width=6).pack(side="left", padx=6)
        ttk.Label(sc_row, text="0-7", foreground=ACCENT,
                  font=("Menlo", 9)).pack(side="left")
        ttk.Button(sc_row, text="Set",
                   command=lambda: self._send_validated(
                       0x04, var_sc, bits=3)).pack(side="right")

        ttk.Button(grp_op, text="Request Status",
                   command=lambda: self._send_cmd(0xFF, 0)).pack(fill="x", pady=2)

        # -- Signal Processing --
        grp_sp = ttk.LabelFrame(left, text="Signal Processing", padding=10)
        grp_sp.pack(fill="x", pady=(0, 8))

        sp_params = [
            # Format: label, opcode, default, bits, hint
            ("Detect Threshold",  0x03, "10000", 16, "0-65535"),
            ("Gain Shift",        0x16, "0",     4,  "0-15, dir+shift"),
            ("MTI Enable",        0x26, "0",     1,  "0=off, 1=on"),
            ("DC Notch Width",    0x27, "0",     3,  "0-7 bins"),
        ]
        for label, opcode, default, bits, hint in sp_params:
            self._add_param_row(grp_sp, label, opcode, default, bits, hint)

        # MTI quick toggle
        mti_row = ttk.Frame(grp_sp)
        mti_row.pack(fill="x", pady=2)
        ttk.Button(mti_row, text="Enable MTI",
                   command=lambda: self._send_cmd(0x26, 1)).pack(
                       side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(mti_row, text="Disable MTI",
                   command=lambda: self._send_cmd(0x26, 0)).pack(
                       side="left", expand=True, fill="x", padx=(2, 0))

        # -- Diagnostics --
        grp_diag = ttk.LabelFrame(left, text="Diagnostics", padding=10)
        grp_diag.pack(fill="x", pady=(0, 8))

        ttk.Button(grp_diag, text="Run Self-Test",
                   command=lambda: self._send_cmd(0x30, 1)).pack(fill="x", pady=2)
        ttk.Button(grp_diag, text="Read Self-Test Result",
                   command=lambda: self._send_cmd(0x31, 0)).pack(fill="x", pady=2)

        st_frame = ttk.LabelFrame(grp_diag, text="Self-Test Results", padding=6)
        st_frame.pack(fill="x", pady=(4, 0))
        self._st_labels = {}
        for name, default_text in [
            ("busy", "Busy: --"),
            ("flags", "Flags: -----"),
            ("detail", "Detail: 0x--"),
            ("t0", "T0 BRAM: --"),
            ("t1", "T1 CIC:  --"),
            ("t2", "T2 FFT:  --"),
            ("t3", "T3 Arith: --"),
            ("t4", "T4 ADC:  --"),
        ]:
            lbl = ttk.Label(st_frame, text=default_text, font=("Menlo", 9))
            lbl.pack(anchor="w")
            self._st_labels[name] = lbl

        # ── Center column: Waveform Timing ────────────────────────────
        center = ttk.Frame(outer)
        center.grid(row=0, column=1, sticky="nsew", padx=6)

        grp_wf = ttk.LabelFrame(center, text="Waveform Timing", padding=10)
        grp_wf.pack(fill="x", pady=(0, 8))

        wf_params = [
            ("Long Chirp Cycles",   0x10, "3000",  16, "0-65535, rst=3000"),
            ("Long Listen Cycles",  0x11, "13700", 16, "0-65535, rst=13700"),
            ("Guard Cycles",        0x12, "17540", 16, "0-65535, rst=17540"),
            ("Short Chirp Cycles",  0x13, "50",    16, "0-65535, rst=50"),
            ("Short Listen Cycles", 0x14, "17450", 16, "0-65535, rst=17450"),
            ("Chirps Per Elevation", 0x15, "32",    6, "1-32, clamped"),
        ]
        for label, opcode, default, bits, hint in wf_params:
            self._add_param_row(grp_wf, label, opcode, default, bits, hint)

        # ── Right column: Detection (CFAR) + Custom ───────────────────
        right = ttk.Frame(outer)
        right.grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        grp_cfar = ttk.LabelFrame(right, text="Detection (CFAR)", padding=10)
        grp_cfar.pack(fill="x", pady=(0, 8))

        cfar_params = [
            ("CFAR Enable",       0x25, "0",  1,  "0=off, 1=on"),
            ("CFAR Guard Cells",  0x21, "2",  4,  "0-15, rst=2"),
            ("CFAR Train Cells",  0x22, "8",  5,  "1-31, rst=8"),
            ("CFAR Alpha (Q4.4)", 0x23, "48", 8,  "0-255, rst=0x30=3.0"),
            ("CFAR Mode",         0x24, "0",  2,  "0=CA 1=GO 2=SO"),
        ]
        for label, opcode, default, bits, hint in cfar_params:
            self._add_param_row(grp_cfar, label, opcode, default, bits, hint)

        # CFAR quick toggle
        cfar_row = ttk.Frame(grp_cfar)
        cfar_row.pack(fill="x", pady=2)
        ttk.Button(cfar_row, text="Enable CFAR",
                   command=lambda: self._send_cmd(0x25, 1)).pack(
                       side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(cfar_row, text="Disable CFAR",
                   command=lambda: self._send_cmd(0x25, 0)).pack(
                       side="left", expand=True, fill="x", padx=(2, 0))

        # ── AGC (Automatic Gain Control) ──────────────────────────────
        grp_agc = ttk.LabelFrame(right, text="AGC (Auto Gain)", padding=10)
        grp_agc.pack(fill="x", pady=(0, 8))

        agc_params = [
            ("AGC Enable",   0x28, "0",   1, "0=manual, 1=auto"),
            ("AGC Target",   0x29, "200", 8, "0-255, peak target"),
            ("AGC Attack",   0x2A, "1",   4, "0-15, atten step"),
            ("AGC Decay",    0x2B, "1",   4, "0-15, gain-up step"),
            ("AGC Holdoff",  0x2C, "4",   4, "0-15, frames"),
        ]
        for label, opcode, default, bits, hint in agc_params:
            self._add_param_row(grp_agc, label, opcode, default, bits, hint)

        # AGC quick toggle
        agc_row = ttk.Frame(grp_agc)
        agc_row.pack(fill="x", pady=2)
        ttk.Button(agc_row, text="Enable AGC",
                   command=lambda: self._send_cmd(0x28, 1)).pack(
                       side="left", expand=True, fill="x", padx=(0, 2))
        ttk.Button(agc_row, text="Disable AGC",
                   command=lambda: self._send_cmd(0x28, 0)).pack(
                       side="left", expand=True, fill="x", padx=(2, 0))

        # AGC status readback labels
        agc_st = ttk.LabelFrame(grp_agc, text="AGC Status", padding=6)
        agc_st.pack(fill="x", pady=(4, 0))
        self._agc_labels = {}
        for name, default_text in [
            ("enable", "AGC: --"),
            ("gain",   "Gain: --"),
            ("peak",   "Peak: --"),
            ("sat",    "Sat Count: --"),
        ]:
            lbl = ttk.Label(agc_st, text=default_text, font=("Menlo", 9))
            lbl.pack(anchor="w")
            self._agc_labels[name] = lbl

        # ── Custom Command (advanced / debug) ─────────────────────────
        grp_cust = ttk.LabelFrame(right, text="Custom Command", padding=10)
        grp_cust.pack(fill="x", pady=(0, 8))

        r0 = ttk.Frame(grp_cust)
        r0.pack(fill="x", pady=2)
        ttk.Label(r0, text="Opcode (hex)").pack(side="left")
        self._custom_op = tk.StringVar(value="01")
        ttk.Entry(r0, textvariable=self._custom_op, width=8).pack(
            side="left", padx=6)

        r1 = ttk.Frame(grp_cust)
        r1.pack(fill="x", pady=2)
        ttk.Label(r1, text="Value (dec)").pack(side="left")
        self._custom_val = tk.StringVar(value="0")
        ttk.Entry(r1, textvariable=self._custom_val, width=8).pack(
            side="left", padx=6)

        ttk.Button(grp_cust, text="Send",
                   command=self._send_custom).pack(fill="x", pady=2)

        # Column weights
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.columnconfigure(2, weight=1)
        outer.rowconfigure(0, weight=1)

    def _add_param_row(self, parent, label: str, opcode: int,
                       default: str, bits: int, hint: str):
        """Add a single parameter row: label, entry, hint, Set button with validation."""
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label).pack(side="left")
        var = tk.StringVar(value=default)
        self._param_vars[str(opcode)] = var
        ttk.Entry(row, textvariable=var, width=8).pack(side="left", padx=6)
        ttk.Label(row, text=hint, foreground=ACCENT,
                  font=("Menlo", 9)).pack(side="left")
        ttk.Button(row, text="Set",
                   command=lambda: self._send_validated(
                       opcode, var, bits=bits)).pack(side="right")

    def _send_validated(self, opcode: int, var: tk.StringVar, bits: int):
        """Parse, clamp to bit-width, send command, and update the entry."""
        try:
            raw = int(var.get())
        except ValueError:
            log.error(f"Invalid value for opcode 0x{opcode:02X}: {var.get()!r}")
            return
        max_val = (1 << bits) - 1
        clamped = max(0, min(raw, max_val))
        if clamped != raw:
            log.warning(f"Value {raw} clamped to {clamped} "
                        f"({bits}-bit max={max_val}) for opcode 0x{opcode:02X}")
            var.set(str(clamped))
        self._send_cmd(opcode, clamped)

    def _build_replay_tab(self, parent):
        """Replay tab — load file, transport controls, seek slider."""
        # File selection
        file_frame = ttk.LabelFrame(parent, text="Replay Source", padding=10)
        file_frame.pack(fill="x", padx=8, pady=(8, 4))

        self._replay_path_var = tk.StringVar(value="(none)")
        ttk.Label(file_frame, textvariable=self._replay_path_var,
                  font=("Menlo", 9)).pack(side="left", fill="x", expand=True)

        ttk.Button(file_frame, text="Browse File...",
                   command=self._replay_browse_file).pack(side="right", padx=(4, 0))
        ttk.Button(file_frame, text="Browse Dir...",
                   command=self._replay_browse_dir).pack(side="right", padx=(4, 0))

        # Transport controls
        ctrl_frame = ttk.LabelFrame(parent, text="Transport", padding=10)
        ctrl_frame.pack(fill="x", padx=8, pady=4)

        btn_row = ttk.Frame(ctrl_frame)
        btn_row.pack(fill="x", pady=(0, 6))

        self._rp_play_btn = ttk.Button(
            btn_row, text="Play", command=self._replay_play, state="disabled")
        self._rp_play_btn.pack(side="left", padx=2)

        self._rp_pause_btn = ttk.Button(
            btn_row, text="Pause", command=self._replay_pause, state="disabled")
        self._rp_pause_btn.pack(side="left", padx=2)

        self._rp_stop_btn = ttk.Button(
            btn_row, text="Stop", command=self._replay_stop, state="disabled")
        self._rp_stop_btn.pack(side="left", padx=2)

        # Speed selector
        ttk.Label(btn_row, text="Speed:").pack(side="left", padx=(16, 4))
        self._rp_speed_var = tk.StringVar(value="1x")
        speed_combo = ttk.Combobox(
            btn_row, textvariable=self._rp_speed_var,
            values=list(_ReplayController.SPEED_MAP.keys()),
            state="readonly", width=6)
        speed_combo.pack(side="left", padx=2)
        speed_combo.bind("<<ComboboxSelected>>", self._replay_speed_changed)

        # Loop checkbox
        self._rp_loop_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(btn_row, text="Loop",
                        variable=self._rp_loop_var,
                        command=self._replay_loop_changed).pack(side="left", padx=8)

        # Seek slider
        slider_row = ttk.Frame(ctrl_frame)
        slider_row.pack(fill="x")

        self._rp_slider = tk.Scale(
            slider_row, from_=0, to=0, orient="horizontal",
            bg=SURFACE, fg=FG, highlightthickness=0,
            troughcolor=BG2, command=self._replay_seek)
        self._rp_slider.pack(side="left", fill="x", expand=True)

        self._rp_frame_label = ttk.Label(
            slider_row, text="0 / 0", font=("Menlo", 10))
        self._rp_frame_label.pack(side="right", padx=8)

        # Status
        self._rp_status_label = ttk.Label(
            parent, text="No replay loaded", font=("Menlo", 10))
        self._rp_status_label.pack(padx=8, pady=4, anchor="w")

        # Info frame for FPGA controls during replay
        info = ttk.LabelFrame(parent, text="Replay FPGA Controls", padding=10)
        info.pack(fill="x", padx=8, pady=4)
        ttk.Label(
            info,
            text=("When replaying Raw IQ data, FPGA Control tab "
                  "parameters are routed to the SoftwareFPGA.\n"
                  "Changes take effect on the next frame."),
            font=("Menlo", 9), foreground=ACCENT,
        ).pack(anchor="w")

    def _build_agc_tab(self, parent):
        """AGC Monitor tab — real-time strip charts for gain, peak, and saturation."""
        # Top row: AGC status badge + saturation indicator
        top = ttk.Frame(parent)
        top.pack(fill="x", padx=8, pady=(8, 0))

        self._agc_badge = ttk.Label(
            top, text="AGC: --", font=("Menlo", 14, "bold"), foreground=FG)
        self._agc_badge.pack(side="left", padx=(0, 24))

        self._agc_sat_badge = ttk.Label(
            top, text="Saturation: 0", font=("Menlo", 12), foreground=GREEN)
        self._agc_sat_badge.pack(side="left", padx=(0, 24))

        self._agc_gain_value = ttk.Label(
            top, text="Gain: --", font=("Menlo", 12), foreground=ACCENT)
        self._agc_gain_value.pack(side="left", padx=(0, 24))

        self._agc_peak_value = ttk.Label(
            top, text="Peak: --", font=("Menlo", 12), foreground=ACCENT)
        self._agc_peak_value.pack(side="left")

        # Matplotlib figure with 3 stacked subplots sharing x-axis (time)
        self._agc_fig = Figure(figsize=(14, 7), facecolor=BG)
        self._agc_fig.subplots_adjust(
            left=0.07, right=0.98, top=0.95, bottom=0.08,
            hspace=0.30)

        # Subplot 1: FPGA inner-loop gain (4-bit, 0-15)
        self._ax_gain = self._agc_fig.add_subplot(3, 1, 1)
        self._ax_gain.set_facecolor(BG2)
        self._ax_gain.set_title("FPGA AGC Gain (inner loop)", color=FG, fontsize=10)
        self._ax_gain.set_ylabel("Gain Level", color=FG)
        self._ax_gain.set_ylim(-0.5, 15.5)
        self._ax_gain.tick_params(colors=FG)
        self._ax_gain.set_xlim(0, self._agc_history_len)
        self._gain_line, = self._ax_gain.plot(
            [], [], color=ACCENT, linewidth=1.5, label="Gain")
        self._ax_gain.axhline(y=0, color=RED, linewidth=0.5, alpha=0.5, linestyle="--")
        self._ax_gain.axhline(y=15, color=RED, linewidth=0.5, alpha=0.5, linestyle="--")
        for spine in self._ax_gain.spines.values():
            spine.set_color(SURFACE)

        # Subplot 2: Peak magnitude (8-bit, 0-255)
        self._ax_peak = self._agc_fig.add_subplot(3, 1, 2)
        self._ax_peak.set_facecolor(BG2)
        self._ax_peak.set_title("Peak Magnitude", color=FG, fontsize=10)
        self._ax_peak.set_ylabel("Peak (8-bit)", color=FG)
        self._ax_peak.set_ylim(-5, 260)
        self._ax_peak.tick_params(colors=FG)
        self._ax_peak.set_xlim(0, self._agc_history_len)
        self._peak_line, = self._ax_peak.plot(
            [], [], color=YELLOW, linewidth=1.5, label="Peak")
        # AGC target reference line (default 200)
        self._agc_target_line = self._ax_peak.axhline(
            y=200, color=GREEN, linewidth=1.0, alpha=0.7, linestyle="--",
            label="Target (200)")
        self._ax_peak.legend(loc="upper right", fontsize=8,
                             facecolor=BG2, edgecolor=SURFACE,
                             labelcolor=FG)
        for spine in self._ax_peak.spines.values():
            spine.set_color(SURFACE)

        # Subplot 3: Saturation count (8-bit, 0-255) as bar-style fill
        self._ax_sat = self._agc_fig.add_subplot(3, 1, 3)
        self._ax_sat.set_facecolor(BG2)
        self._ax_sat.set_title("Saturation Count", color=FG, fontsize=10)
        self._ax_sat.set_ylabel("Sat Count", color=FG)
        self._ax_sat.set_xlabel("Sample Index", color=FG)
        self._ax_sat.set_ylim(-1, 40)
        self._ax_sat.tick_params(colors=FG)
        self._ax_sat.set_xlim(0, self._agc_history_len)
        self._sat_fill = self._ax_sat.fill_between(
            [], [], color=RED, alpha=0.6, label="Saturation")
        self._sat_line, = self._ax_sat.plot(
            [], [], color=RED, linewidth=1.0)
        self._ax_sat.axhline(y=0, color=GREEN, linewidth=0.5, alpha=0.5, linestyle="--")
        for spine in self._ax_sat.spines.values():
            spine.set_color(SURFACE)

        agc_canvas = FigureCanvasTkAgg(self._agc_fig, master=parent)
        agc_canvas.draw()
        agc_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._agc_canvas = agc_canvas

    def _build_log_tab(self, parent):
        self.log_text = tk.Text(parent, bg=BG2, fg=FG, font=("Menlo", 10),
                                 insertbackground=FG, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

        # Redirect log handler to text widget (via UI queue for thread safety)
        handler = _TextHandler(self._ui_queue)
        handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                                                datefmt="%H:%M:%S"))
        logging.getLogger().addHandler(handler)

    # ------------------------------------------------------------ Actions
    def _on_connect(self):
        if self.conn is not None and self.conn.is_open:
            # Disconnect
            if self._acq_thread is not None:
                self._acq_thread.stop()
                self._acq_thread.join(timeout=2)
                self._acq_thread = None
            self.conn.close()
            self.conn = None
            self.lbl_status.config(text="DISCONNECTED", foreground=RED)
            self.btn_connect.config(text="Connect")
            self.cmb_usb_iface.config(state="readonly")
            log.info("Disconnected")
            return

        # Stop any active demo or replay before going live
        if self._demo_active:
            self._stop_demo()
        if self._replay_active:
            self._replay_stop()

        # Create connection based on USB Interface selector
        iface = self._usb_iface_var.get()
        if "FT601" in iface:
            self.conn = FT601Connection(mock=self._mock)
        else:
            self.conn = FT2232HConnection(mock=self._mock)

        # Disable interface selector while connecting/connected
        self.cmb_usb_iface.config(state="disabled")

        # Open connection in a background thread to avoid blocking the GUI
        self.lbl_status.config(text="CONNECTING...", foreground=YELLOW)
        self.btn_connect.config(state="disabled")
        self.root.update_idletasks()

        def _do_connect():
            ok = self.conn.open(self.device_index)
            # Post result to UI queue (drained by _schedule_update)
            self._ui_queue.put(("connect", ok))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _on_connect_done(self, success: bool):
        """Called on main thread after connection attempt completes."""
        self.btn_connect.config(state="normal")
        if success:
            self.lbl_status.config(text="CONNECTED", foreground=GREEN)
            self.btn_connect.config(text="Disconnect")
            self._acq_thread = RadarAcquisition(
                self.conn, self.frame_queue, self.recorder,
                status_callback=self._on_status_received)
            self._acq_thread.start()
            log.info("Connected and acquisition started")
        else:
            self.lbl_status.config(text="CONNECT FAILED", foreground=RED)
            self.btn_connect.config(text="Connect")
            self.cmb_usb_iface.config(state="readonly")
            self.conn = None

    def _on_record(self):
        if self.recorder.recording:
            self.recorder.stop()
            self.btn_record.config(text="Record")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".h5",
            filetypes=[("HDF5", "*.h5"), ("All", "*.*")],
            initialfile=f"radar_{time.strftime('%Y%m%d_%H%M%S')}.h5",
        )
        if filepath:
            self.recorder.start(filepath)
            self.btn_record.config(text="Stop Rec")

    # Opcode → SoftwareFPGA setter method name for dual dispatch during replay
    _SFPGA_SETTER_NAMES: ClassVar[dict[int, str]] = {
        0x03: "set_detect_threshold",
        0x16: "set_gain_shift",
        0x21: "set_cfar_guard",
        0x22: "set_cfar_train",
        0x23: "set_cfar_alpha",
        0x24: "set_cfar_mode",
        0x25: "set_cfar_enable",
        0x26: "set_mti_enable",
        0x27: "set_dc_notch_width",
        0x28: "set_agc_enable",
    }

    def _send_cmd(self, opcode: int, value: int):
        """Send command — routes to SoftwareFPGA when replaying raw IQ."""
        if (self._replay_active and self._replay_ctrl is not None
                and self._replay_ctrl.software_fpga is not None):
            sfpga = self._replay_ctrl.software_fpga
            setter_name = self._SFPGA_SETTER_NAMES.get(opcode)
            if setter_name is not None:
                getattr(sfpga, setter_name)(value)
                log.info(
                    f"SoftwareFPGA 0x{opcode:02X} val={value}")
                return
            log.warning(
                f"Opcode 0x{opcode:02X} not routable in replay mode")
            self._ui_queue.put(
                ("status_msg",
                 f"Opcode 0x{opcode:02X} is hardware-only (ignored in replay)"))
            return
        cmd = RadarProtocol.build_command(opcode, value)
        if self.conn is None:
            log.warning("No connection — command not sent")
            return
        ok = self.conn.write(cmd)
        log.info(f"CMD 0x{opcode:02X} val={value} ({'OK' if ok else 'FAIL'})")

    def _send_custom(self):
        try:
            op = int(self._custom_op.get(), 16)
            val = int(self._custom_val.get())
            self._send_cmd(op, val)
        except ValueError:
            log.error("Invalid custom command values")

    # -------------------------------------------------------- Replay actions
    def _replay_browse_file(self):
        path = filedialog.askopenfilename(
            title="Select replay file",
            filetypes=[
                ("NumPy files", "*.npy"),
                ("HDF5 files", "*.h5"),
                ("All files", "*.*"),
            ],
        )
        if path:
            self._replay_load(path)

    def _replay_browse_dir(self):
        path = filedialog.askdirectory(title="Select co-sim directory")
        if path:
            self._replay_load(path)

    def _replay_load(self, path: str):
        """Load replay data and enable transport controls."""
        # Stop any running mode
        if self._demo_active:
            self._stop_demo()
        # Safely shutdown and disable UI controls before loading the new file
        if self._replay_active or self._replay_ctrl is not None:
            self._replay_stop()
        if self._acq_thread is not None:
            if self.conn is not None and self.conn.is_open:
                self._on_connect()  # disconnect
            else:
                # Connection dropped unexpectedly — just clean up the thread
                self._acq_thread.stop()
                self._acq_thread.join(timeout=2)
                self._acq_thread = None

        try:
            self._replay_ctrl = _ReplayController(
                self.frame_queue, self._ui_queue)
            total = self._replay_ctrl.load(path)
        except Exception as exc:  # noqa: BLE001
            log.error(f"Failed to load replay: {exc}")
            self._rp_status_label.config(
                text=f"Load failed: {exc}", foreground=RED)
            self._replay_ctrl = None
            return

        short_path = Path(path).name
        self._replay_path_var.set(short_path)
        self._rp_slider.config(to=max(0, total - 1))
        self._rp_frame_label.config(text=f"0 / {total}")
        self._rp_status_label.config(
            text=f"Loaded: {total} frames from {short_path}",
            foreground=GREEN)

        # Enable transport buttons
        for btn in (self._rp_play_btn, self._rp_pause_btn, self._rp_stop_btn):
            btn.config(state="normal")

        self._replay_active = True
        self.lbl_status.config(text="REPLAY", foreground=ACCENT)
        log.info(f"Replay loaded: {total} frames from {path}")

    def _replay_play(self):
        if self._replay_ctrl:
            self._replay_ctrl.play()

    def _replay_pause(self):
        if self._replay_ctrl:
            self._replay_ctrl.pause()

    def _replay_stop(self):
        if self._replay_ctrl:
            self._replay_ctrl.close()
            self._replay_ctrl = None
        self._replay_active = False
        self.lbl_status.config(text="DISCONNECTED", foreground=RED)
        self._rp_slider.set(0)
        self._rp_frame_label.config(text="0 / 0")
        for btn in (self._rp_play_btn, self._rp_pause_btn, self._rp_stop_btn):
            btn.config(state="disabled")

    def _replay_seek(self, value):
        if (self._replay_ctrl and self._replay_active
                and not self._replay_ctrl.is_playing):
            self._replay_ctrl.seek(int(value))

    def _replay_speed_changed(self, _event=None):
        if self._replay_ctrl:
            self._replay_ctrl.set_speed(self._rp_speed_var.get())

    def _replay_loop_changed(self):
        if self._replay_ctrl:
            self._replay_ctrl.set_loop(self._rp_loop_var.get())

    # ---------------------------------------------------------- Demo actions
    def _toggle_demo(self):
        if self._demo_active:
            self._stop_demo()
        else:
            self._start_demo()

    def _start_demo(self):
        """Start demo mode with synthetic targets."""
        # Mutual exclusion
        if self._replay_active:
            self._replay_stop()
        if self._acq_thread is not None:
            log.warning("Cannot start demo while radar is connected")
            return

        n = max(1, min(DEMO_N_MAX, self._demo_n))
        self._demo_sim = DemoSimulator(
            self.frame_queue, self._ui_queue, self.root,
            interval_ms=DEMO_INTERVAL_MS, n_targets=n, on_ppi=self._blit_ppi)
        self._demo_sim.start()
        self._demo_active = True
        self.lbl_status.config(text=f"DEMO {n}", foreground=YELLOW)
        self.btn_demo.config(text="Stop Demo")
        log.info("Demo mode started (%d targets)", n)

    def _stop_demo(self):
        if self._demo_sim is not None:
            self._demo_sim.stop()
            self._demo_sim = None
        self._demo_active = False
        self._detected_targets = []
        self._update_targets_table([])
        self._update_ppi([])
        self.lbl_status.config(text="DISCONNECTED", foreground=RED)
        self.btn_demo.config(text="Start Demo")
        log.info("Demo mode stopped")

    def _on_status_received(self, status: StatusResponse):
        """Called from acquisition thread — post to UI queue for main thread."""
        self._ui_queue.put(("status", status))

    def _update_self_test_labels(self, status: StatusResponse):
        """Update the self-test result labels and AGC status from a StatusResponse."""
        if not hasattr(self, '_st_labels'):
            return
        flags = status.self_test_flags
        detail = status.self_test_detail
        busy = status.self_test_busy

        busy_str = "RUNNING" if busy else "IDLE"
        busy_color = YELLOW if busy else FG
        self._st_labels["busy"].config(text=f"Busy: {busy_str}",
                                        foreground=busy_color)
        self._st_labels["flags"].config(text=f"Flags: {flags:05b}")
        self._st_labels["detail"].config(text=f"Detail: 0x{detail:02X}")

        # Individual test results (bit = 1 means PASS)
        test_names = [
            ("t0", "T0 BRAM"),
            ("t1", "T1 CIC"),
            ("t2", "T2 FFT"),
            ("t3", "T3 Arith"),
            ("t4", "T4 ADC"),
        ]
        for i, (key, name) in enumerate(test_names):
            if busy:
                result_str = "..."
                color = YELLOW
            elif flags & (1 << i):
                result_str = "PASS"
                color = GREEN
            else:
                result_str = "FAIL"
                color = RED
            self._st_labels[key].config(
                text=f"{name}: {result_str}", foreground=color)

        # AGC status readback
        if hasattr(self, '_agc_labels'):
            agc_str = "AUTO" if status.agc_enable else "MANUAL"
            agc_color = GREEN if status.agc_enable else FG
            self._agc_labels["enable"].config(
                text=f"AGC: {agc_str}", foreground=agc_color)
            self._agc_labels["gain"].config(
                text=f"Gain: {status.agc_current_gain}")
            self._agc_labels["peak"].config(
                text=f"Peak: {status.agc_peak_magnitude}")
            sat_color = RED if status.agc_saturation_count > 0 else FG
            self._agc_labels["sat"].config(
                text=f"Sat Count: {status.agc_saturation_count}",
                foreground=sat_color)

        # AGC visualization update
        self._update_agc_visualization(status)

    def _update_agc_visualization(self, status: StatusResponse):
        """Push AGC metrics into ring buffers and redraw strip charts.

        Data is always accumulated (cheap), but matplotlib redraws are
        throttled to ``_AGC_REDRAW_INTERVAL`` seconds to avoid saturating
        the GUI event-loop when status packets arrive at 20 Hz.
        """
        if not hasattr(self, '_agc_canvas'):
            return

        # Append to ring buffers (always — this is O(1))
        self._agc_gain_history.append(status.agc_current_gain)
        self._agc_peak_history.append(status.agc_peak_magnitude)
        self._agc_sat_history.append(status.agc_saturation_count)

        # Update indicator labels (cheap Tk config calls)
        mode_str = "AUTO" if status.agc_enable else "MANUAL"
        mode_color = GREEN if status.agc_enable else FG
        self._agc_badge.config(text=f"AGC: {mode_str}", foreground=mode_color)
        self._agc_gain_value.config(
            text=f"Gain: {status.agc_current_gain}")
        self._agc_peak_value.config(
            text=f"Peak: {status.agc_peak_magnitude}")

        total_sat = sum(self._agc_sat_history)
        if total_sat > 10:
            sat_color = RED
        elif total_sat > 0:
            sat_color = YELLOW
        else:
            sat_color = GREEN
        self._agc_sat_badge.config(
            text=f"Saturation: {total_sat}", foreground=sat_color)

        # ---- Throttle matplotlib redraws ---------------------------------
        now = time.monotonic()
        if now - self._agc_last_redraw < self._AGC_REDRAW_INTERVAL:
            return
        self._agc_last_redraw = now

        n = len(self._agc_gain_history)
        xs = list(range(n))

        # Update line plots
        gain_data = list(self._agc_gain_history)
        peak_data = list(self._agc_peak_history)
        sat_data = list(self._agc_sat_history)

        self._gain_line.set_data(xs, gain_data)
        self._peak_line.set_data(xs, peak_data)

        # Saturation: redraw as filled area
        self._sat_line.set_data(xs, sat_data)
        if self._sat_fill is not None:
            self._sat_fill.remove()
        self._sat_fill = self._ax_sat.fill_between(
            xs, sat_data, color=RED, alpha=0.4)

        # Auto-scale saturation Y axis to data
        max_sat = max(sat_data) if sat_data else 0
        self._ax_sat.set_ylim(-1, max(max_sat * 1.5, 5))

        # Scroll X axis to keep latest data visible
        if n >= self._agc_history_len:
            self._ax_gain.set_xlim(0, n)
            self._ax_peak.set_xlim(0, n)
            self._ax_sat.set_xlim(0, n)

        self._agc_canvas.draw_idle()

    # --------------------------------------------------------- Display loop
    def _schedule_update(self):
        self._drain_ui_queue()
        self._update_display()
        self.root.after(self.UPDATE_INTERVAL_MS, self._schedule_update)

    def _drain_ui_queue(self):
        """Process all pending cross-thread messages on the main thread."""
        while True:
            try:
                tag, payload = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            if tag == "connect":
                self._on_connect_done(payload)
            elif tag == "status":
                self._update_self_test_labels(payload)
            elif tag == "log":
                self._log_handler_append(payload)
            elif tag == "replay_state":
                self._on_replay_state(payload)
            elif tag == "replay_index":
                self._on_replay_index(*payload)
            elif tag == "demo_targets":
                self._on_demo_targets(payload)
            elif tag == "status_msg":
                self.lbl_status.config(text=str(payload), foreground=YELLOW)

    def _on_replay_state(self, state: str):
        if state == "playing":
            self._rp_status_label.config(text="Playing", foreground=GREEN)
        elif state == "paused":
            self._rp_status_label.config(text="Paused", foreground=YELLOW)
        elif state == "stopped":
            self._rp_status_label.config(text="Stopped", foreground=FG)

    def _on_replay_index(self, index: int, total: int):
        self._rp_frame_label.config(text=f"{index} / {total}")
        self._rp_slider.set(index)

    def _on_demo_targets(self, targets: list[dict]):
        """Table only — PPI is blitted from numpy arrays in ``_blit_ppi``."""
        self._detected_targets = targets
        self._update_targets_table(targets)

    def _blit_ppi(self, range_m, azimuth_deg):
        """Stamp blips under the rotating sweep, phosphor-decay the rest."""
        sweep_az = (time.monotonic() * 360.0 * PPI_SWEEP_RPS) % 360.0
        rad = np.radians(sweep_az)
        reach = self._max_range
        self._sweep_line.set_data(
            [0.0, reach * np.sin(rad)], [0.0, reach * np.cos(rad)])
        az = np.asarray(azimuth_deg, dtype=np.float64)
        ppi_raster(
            ppi_xy(range_m, az), self._max_range,
            out=self._ppi_buf, decay=PPI_DECAY,
            weights=ppi_beam_weights(az, sweep_az),
        )
        self._ppi_img.set_data(self._ppi_buf)

    def _update_ppi(self, targets: list[dict]):
        """Clear or (slow path) stamp from dict rows. Demo uses ``_blit_ppi``."""
        if not targets:
            self._ppi_buf.fill(0.0)
            self._ppi_img.set_data(self._ppi_buf)
            self._canvas.draw_idle()
            return
        self._blit_ppi(
            [t.get("range_m", 0.0) for t in targets],
            [t.get("azimuth", 0.0) for t in targets],
        )

    def _update_targets_table(self, targets: list[dict]):
        """Refresh the detected targets treeview."""
        # Clear existing rows
        for item in self._tgt_tree.get_children():
            self._tgt_tree.delete(item)
        # Insert new rows
        for t in targets:
            self._tgt_tree.insert("", "end", values=(
                t.get("id", ""),
                f"{t.get('range_m', 0):.0f}",
                f"{t.get('velocity', 0):.1f}",
                f"{t.get('azimuth', 0):.1f}",
                f"{t.get('snr', 0):.1f}",
                t.get("class", ""),
            ))

    def _log_handler_append(self, msg: str):
        """Append a log message to the log Text widget (main thread only)."""
        with contextlib.suppress(Exception):
            self.log_text.insert("end", msg + "\n")
            self.log_text.see("end")
            # Keep last 500 lines
            lines = int(self.log_text.index("end-1c").split(".")[0])
            if lines > 500:
                self.log_text.delete("1.0", f"{lines - 500}.0")

    def _update_display(self):
        """Pull latest frame from queue and update plots."""
        frame = None
        # Drain queue, keep latest
        while True:
            try:
                frame = self.frame_queue.get_nowait()
            except queue.Empty:
                break

        if frame is None:
            return

        self._current_frame = frame
        self._frame_count += 1

        # FPS calculation
        now = time.time()
        dt = now - self._fps_ts
        if dt > 0.5:
            self._fps = self._frame_count / dt
            self._frame_count = 0
            self._fps_ts = now

        # Update labels
        self.lbl_fps.config(text=f"{self._fps:.1f} fps")
        self.lbl_detections.config(text=f"Det: {frame.detection_count}")
        self.lbl_frame.config(text=f"Frame: {frame.frame_number}")

        # Update range-Doppler heatmap in raw dual-subframe bin order
        mag = frame.magnitude
        det_shifted = frame.detections

        # Stable colorscale via EMA smoothing of vmax
        frame_vmax = float(np.max(mag)) if np.max(mag) > 0 else 1.0
        self._vmax_ema = (self._vmax_alpha * frame_vmax +
                          (1.0 - self._vmax_alpha) * self._vmax_ema)
        stable_vmax = max(self._vmax_ema, 1.0)

        self._rd_img.set_data(mag)
        self._rd_img.set_clim(vmin=0, vmax=stable_vmax)

        # Update CFAR overlay in raw Doppler-bin coordinates
        det_coords = np.argwhere(det_shifted > 0)
        if len(det_coords) > 0:
            # det_coords[:, 0] = range bin, det_coords[:, 1] = Doppler bin
            range_m = (det_coords[:, 0] + 0.5) * self._range_per_bin
            doppler_bins = det_coords[:, 1] + 0.5
            offsets = np.column_stack([doppler_bins, range_m])
            self._det_scatter.set_offsets(offsets)
        else:
            self._det_scatter.set_offsets(np.empty((0, 2)))

        # Update waterfall
        self._waterfall.append(frame.range_profile.copy())
        wf_arr = np.array(list(self._waterfall))
        wf_max = max(np.max(wf_arr), 1.0)
        self._wf_img.set_data(wf_arr)
        self._wf_img.set_clim(vmin=0, vmax=wf_max)

        self._canvas.draw_idle()


class _TextHandler(logging.Handler):
    """Logging handler that posts messages to a queue for main-thread append.

    Using widget.after() from background threads crashes Python 3.12 due to
    GIL state corruption.  Instead we post to the dashboard's _ui_queue and
    let _drain_ui_queue() append on the main thread.
    """

    def __init__(self, ui_queue: queue.Queue[tuple[str, object]]):
        super().__init__()
        self._ui_queue = ui_queue

    def emit(self, record):
        msg = self.format(record)
        with contextlib.suppress(Exception):
            self._ui_queue.put(("log", msg))


# ============================================================================
# Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="AERIS-10 Radar Dashboard")
    parser.add_argument("--record", action="store_true",
                        help="Start HDF5 recording immediately")
    parser.add_argument("--device", type=int, default=0,
                        help="FT2232H device index (default: 0)")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--live", action="store_true",
                            help="Use real FT2232H hardware (default: mock mode)")
    mode_group.add_argument("--replay", type=str, default=None,
                            help="Auto-load replay file or directory on startup")
    mode_group.add_argument("--demo", action="store_true",
                            help="Start in demo mode with synthetic targets")
    parser.add_argument("--demo-n", type=int, default=DEMO_N_DEFAULT,
                        help=f"Demo target count (default: {DEMO_N_DEFAULT}, max {DEMO_N_MAX})")
    args = parser.parse_args()

    if args.live:
        mock = False
        mode_str = "LIVE"
    else:
        mock = True
        mode_str = "MOCK"

    recorder = DataRecorder()

    root = tk.Tk()

    dashboard = RadarDashboard(root, mock, recorder, device_index=args.device)
    dashboard._demo_n = max(1, min(DEMO_N_MAX, args.demo_n))

    if args.record:
        filepath = os.path.join(
            os.getcwd(),
            f"radar_{time.strftime('%Y%m%d_%H%M%S')}.h5"
        )
        recorder.start(filepath)

    if args.replay:
        dashboard._replay_load(args.replay)

    if args.demo:
        dashboard._start_demo()

    def on_closing():
        # Stop demo if active
        if dashboard._demo_active:
            dashboard._stop_demo()
        # Stop replay if active
        if dashboard._replay_ctrl is not None:
            dashboard._replay_ctrl.close()
        if dashboard._acq_thread is not None:
            dashboard._acq_thread.stop()
            dashboard._acq_thread.join(timeout=2)
        if dashboard.conn is not None and dashboard.conn.is_open:
            dashboard.conn.close()
        if recorder.recording:
            recorder.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    log.info(f"Dashboard started (mode={mode_str})")
    root.mainloop()


if __name__ == "__main__":
    main()
