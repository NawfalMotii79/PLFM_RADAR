#!/usr/bin/env python3
"""
AERIS-10 Radar Dashboard — Board Bring-Up Edition
===================================================
Real-time visualization and control for the AERIS-10 phased-array radar
via FT2232H USB 2.0 interface.

Features:
  - FT2232H USB reader with packet parsing (matches usb_data_interface_ft2232h.v)
  - Real-time range-Doppler magnitude heatmap (64x32)
  - CFAR detection overlay (flagged cells highlighted)
  - Range profile waterfall plot (range vs. time)
  - Host command sender (opcodes 0x01-0x27, 0x30, 0xFF)
  - Configuration panel for all radar parameters
  - HDF5 data recording for offline analysis
  - Mock mode for development/testing without hardware

Usage:
  python radar_dashboard.py              # Launch with mock data
  python radar_dashboard.py --live       # Launch with FT2232H hardware
  python radar_dashboard.py --record     # Launch with HDF5 recording
"""

import os
import time
import queue
import logging
import argparse
import threading
from typing import Optional, Dict
from collections import deque

import numpy as np

import tkinter as tk
from tkinter import ttk, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Import protocol layer (no GUI deps)
from radar_protocol import (
    RadarProtocol, FT2232HConnection, ReplayConnection,
    DataRecorder, RadarAcquisition,
    RadarFrame, StatusResponse,
    NUM_RANGE_BINS, NUM_DOPPLER_BINS, WATERFALL_DEPTH,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("radar_dashboard")



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


class RadarDashboard:
    """Main tkinter application: real-time radar visualization and control."""

    UPDATE_INTERVAL_MS = 100  # 10 Hz display refresh

    # Radar parameters used for range-axis scaling.
    BANDWIDTH = 500e6        # Hz — chirp bandwidth
    C = 3e8                  # m/s — speed of light

    def __init__(self, root: tk.Tk, connection: FT2232HConnection,
                 recorder: DataRecorder, device_index: int = 0):
        self.root = root
        self.conn = connection
        self.recorder = recorder
        self._device_index = device_index

        self.root.title("AERIS-10 Radar Dashboard — Bring-Up Edition")
        self.root.geometry("1600x950")
        self.root.configure(bg=BG)

        # Frame queue (acquisition → display)
        self.frame_queue: queue.Queue[RadarFrame] = queue.Queue(maxsize=8)
        self._acq_thread: Optional[RadarAcquisition] = None

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

        self.btn_record = ttk.Button(top, text="Record", command=self._on_record)
        self.btn_record.pack(side="right", padx=4)

        # Notebook (tabs)
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        tab_display = ttk.Frame(nb)
        tab_control = ttk.Frame(nb)
        tab_log = ttk.Frame(nb)
        nb.add(tab_display, text="  Display  ")
        nb.add(tab_control, text="  Control  ")
        nb.add(tab_log, text="  Log  ")

        self._build_display_tab(tab_display)
        self._build_control_tab(tab_control)
        self._build_log_tab(tab_log)

    def _build_display_tab(self, parent):
        # Compute physical axis limits
        # Range resolution: dR = c / (2 * BW) per range bin
        # But we decimate 1024→64 bins, so each bin spans 16 FFT bins.
        # Range per FFT bin = c / (2 * BW) * (Fs / FFT_SIZE) — simplified:
        #   max_range = c * Fs / (4 * BW) for Fs-sampled baseband
        #   range_per_bin = max_range / NUM_RANGE_BINS
        range_res = self.C / (2.0 * self.BANDWIDTH)  # ~0.3 m per FFT bin
        # After decimation 1024→64, each range bin = 16 FFT bins
        range_per_bin = range_res * 16
        max_range = range_per_bin * NUM_RANGE_BINS

        doppler_bin_lo = 0
        doppler_bin_hi = NUM_DOPPLER_BINS

        # Matplotlib figure with 3 subplots
        self.fig = Figure(figsize=(14, 7), facecolor=BG)
        self.fig.subplots_adjust(left=0.07, right=0.98, top=0.94, bottom=0.10,
                                  wspace=0.30, hspace=0.35)

        # Range-Doppler heatmap
        self.ax_rd = self.fig.add_subplot(1, 3, (1, 2))
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
        self.ax_wf = self.fig.add_subplot(1, 3, 3)
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

        canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    def _build_control_tab(self, parent):
        """Host command sender and configuration panel."""
        outer = ttk.Frame(parent)
        outer.pack(fill="both", expand=True, padx=16, pady=16)

        # Left column: Quick actions
        left = ttk.LabelFrame(outer, text="Quick Actions", padding=12)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        ttk.Button(left, text="Trigger Chirp (0x01)",
                   command=lambda: self._send_cmd(0x01, 1)).pack(fill="x", pady=3)
        ttk.Button(left, text="Enable MTI (0x26)",
                   command=lambda: self._send_cmd(0x26, 1)).pack(fill="x", pady=3)
        ttk.Button(left, text="Disable MTI (0x26)",
                   command=lambda: self._send_cmd(0x26, 0)).pack(fill="x", pady=3)
        ttk.Button(left, text="Enable CFAR (0x25)",
                   command=lambda: self._send_cmd(0x25, 1)).pack(fill="x", pady=3)
        ttk.Button(left, text="Disable CFAR (0x25)",
                   command=lambda: self._send_cmd(0x25, 0)).pack(fill="x", pady=3)
        ttk.Button(left, text="Request Status (0xFF)",
                   command=lambda: self._send_cmd(0xFF, 0)).pack(fill="x", pady=3)

        ttk.Separator(left, orient="horizontal").pack(fill="x", pady=6)

        ttk.Label(left, text="FPGA Self-Test", font=("Menlo", 10, "bold")).pack(
            anchor="w", pady=(2, 0))
        ttk.Button(left, text="Run Self-Test (0x30)",
                   command=lambda: self._send_cmd(0x30, 1)).pack(fill="x", pady=3)
        ttk.Button(left, text="Read Self-Test Result (0x31)",
                   command=lambda: self._send_cmd(0x31, 0)).pack(fill="x", pady=3)

        # Self-test result display
        st_frame = ttk.LabelFrame(left, text="Self-Test Results", padding=6)
        st_frame.pack(fill="x", pady=(6, 0))
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

        # Right column: Parameter configuration
        right = ttk.LabelFrame(outer, text="Parameter Configuration", padding=12)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self._param_vars: Dict[str, tk.StringVar] = {}
        params = [
            ("CFAR Guard (0x21)", 0x21, "2"),
            ("CFAR Train (0x22)", 0x22, "8"),
            ("CFAR Alpha Q4.4 (0x23)", 0x23, "48"),
            ("CFAR Mode (0x24)", 0x24, "0"),
            ("Threshold (0x10)", 0x10, "500"),
            ("Gain Shift (0x06)", 0x06, "0"),
            ("DC Notch Width (0x27)", 0x27, "0"),
            ("Range Mode (0x20)", 0x20, "0"),
            ("Stream Enable (0x05)", 0x05, "7"),
        ]

        for row_idx, (label, opcode, default) in enumerate(params):
            ttk.Label(right, text=label).grid(row=row_idx, column=0,
                                               sticky="w", pady=2)
            var = tk.StringVar(value=default)
            self._param_vars[str(opcode)] = var
            ent = ttk.Entry(right, textvariable=var, width=10)
            ent.grid(row=row_idx, column=1, padx=8, pady=2)
            ttk.Button(
                right, text="Set",
                command=lambda op=opcode, v=var: self._send_cmd(op, int(v.get()))
            ).grid(row=row_idx, column=2, pady=2)

        # Custom command
        ttk.Separator(right, orient="horizontal").grid(
            row=len(params), column=0, columnspan=3, sticky="ew", pady=8)

        ttk.Label(right, text="Custom Opcode (hex)").grid(
            row=len(params) + 1, column=0, sticky="w")
        self._custom_op = tk.StringVar(value="01")
        ttk.Entry(right, textvariable=self._custom_op, width=10).grid(
            row=len(params) + 1, column=1, padx=8)

        ttk.Label(right, text="Value (dec)").grid(
            row=len(params) + 2, column=0, sticky="w")
        self._custom_val = tk.StringVar(value="0")
        ttk.Entry(right, textvariable=self._custom_val, width=10).grid(
            row=len(params) + 2, column=1, padx=8)

        ttk.Button(right, text="Send Custom",
                   command=self._send_custom).grid(
            row=len(params) + 2, column=2, pady=2)

        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(0, weight=1)

    def _build_log_tab(self, parent):
        self.log_text = tk.Text(parent, bg=BG2, fg=FG, font=("Menlo", 10),
                                 insertbackground=FG, wrap="word")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

        # Redirect log handler to text widget
        handler = _TextHandler(self.log_text)
        handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s",
                                                datefmt="%H:%M:%S"))
        logging.getLogger().addHandler(handler)

    # ------------------------------------------------------------ Actions
    def _on_connect(self):
        if self.conn.is_open:
            # Disconnect
            if self._acq_thread is not None:
                self._acq_thread.stop()
                self._acq_thread.join(timeout=2)
                self._acq_thread = None
            self.conn.close()
            self.lbl_status.config(text="DISCONNECTED", foreground=RED)
            self.btn_connect.config(text="Connect")
            log.info("Disconnected")
            return

        # Open connection in a background thread to avoid blocking the GUI
        self.lbl_status.config(text="CONNECTING...", foreground=YELLOW)
        self.btn_connect.config(state="disabled")
        self.root.update_idletasks()

        def _do_connect():
            ok = self.conn.open(device_index=self._device_index)
            # Schedule UI update back on the main thread
            self.root.after(0, lambda: self._on_connect_done(ok))

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

    def _send_cmd(self, opcode: int, value: int):
        cmd = RadarProtocol.build_command(opcode, value)
        ok = self.conn.write(cmd)
        log.info(f"CMD 0x{opcode:02X} val={value} ({'OK' if ok else 'FAIL'})")

    def _send_custom(self):
        try:
            op = int(self._custom_op.get(), 16)
            val = int(self._custom_val.get())
            self._send_cmd(op, val)
        except ValueError:
            log.error("Invalid custom command values")

    def _on_status_received(self, status: StatusResponse):
        """Called from acquisition thread — schedule UI update on main thread."""
        self.root.after(0, self._update_self_test_labels, status)

    def _update_self_test_labels(self, status: StatusResponse):
        """Update the self-test result labels from a StatusResponse."""
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

    # --------------------------------------------------------- Display loop
    def _schedule_update(self):
        self._update_display()
        self.root.after(self.UPDATE_INTERVAL_MS, self._schedule_update)

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
    """Logging handler that writes to a tkinter Text widget."""

    def __init__(self, text_widget: tk.Text):
        super().__init__()
        self._text = text_widget

    def emit(self, record):
        msg = self.format(record)
        try:
            self._text.after(0, self._append, msg)
        except Exception:
            pass

    def _append(self, msg: str):
        self._text.insert("end", msg + "\n")
        self._text.see("end")
        # Keep last 500 lines
        lines = int(self._text.index("end-1c").split(".")[0])
        if lines > 500:
            self._text.delete("1.0", f"{lines - 500}.0")


# ============================================================================
# Entry Point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="AERIS-10 Radar Dashboard")
    parser.add_argument("--live", action="store_true",
                        help="Use real FT2232H hardware (default: mock mode)")
    parser.add_argument("--replay", type=str, metavar="NPY_DIR",
                        help="Replay real data from .npy directory "
                             "(e.g. tb/cosim/real_data/hex/)")
    parser.add_argument("--no-mti", action="store_true",
                        help="With --replay, use non-MTI Doppler data")
    parser.add_argument("--record", action="store_true",
                        help="Start HDF5 recording immediately")
    parser.add_argument("--device", type=int, default=0,
                        help="FT2232H device index (default: 0)")
    args = parser.parse_args()

    if args.replay:
        npy_dir = os.path.abspath(args.replay)
        conn = ReplayConnection(npy_dir, use_mti=not args.no_mti)
        mode_str = f"REPLAY ({npy_dir}, MTI={'OFF' if args.no_mti else 'ON'})"
    elif args.live:
        conn = FT2232HConnection(mock=False)
        mode_str = f"LIVE (device={args.device})"
    else:
        conn = FT2232HConnection(mock=True)
        mode_str = "MOCK"

    recorder = DataRecorder()

    root = tk.Tk()

    dashboard = RadarDashboard(root, conn, recorder, device_index=args.device)

    if args.record:
        filepath = os.path.join(
            os.getcwd(),
            f"radar_{time.strftime('%Y%m%d_%H%M%S')}.h5"
        )
        recorder.start(filepath)

    def on_closing():
        if dashboard._acq_thread is not None:
            dashboard._acq_thread.stop()
            dashboard._acq_thread.join(timeout=2)
        if conn.is_open:
            conn.close()
        if recorder.recording:
            recorder.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    log.info(f"Dashboard started (mode={mode_str})")
    root.mainloop()


if __name__ == "__main__":
    main()
