#!/usr/bin/env python3
"""
Live 1000 Hz plot for the Bertec AM6817-E force plate.

Acquires all 1 kHz samples on a background thread and renders a scrolling
real-time plot using matplotlib blitting. Press 't' in the plot window to tare.
"""

from __future__ import annotations

import argparse
from collections import deque
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from bertec_am6817 import BertecAM6817


class LivePlotReader:
    """Background 1000 Hz reader feeding a thread-safe ring buffer."""

    def __init__(self, dev: BertecAM6817, max_samples: int) -> None:
        self.dev = dev
        self.max_samples = max_samples
        self.lock = threading.Lock()
        self.t_buf: deque[float] = deque(maxlen=max_samples)
        self.fm_buf: deque[tuple[float, float, float, float, float, float]] = deque(maxlen=max_samples)
        self.sample_count = 0
        self.dropped = 0
        self.measured_hz = 0.0
        self._stop = threading.Event()
        self._tare_req = threading.Event()
        self._tare_samples = 200
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.dev.start_stream()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.dev.stop_stream()

    def request_tare(self, n_samples: int = 200) -> None:
        self._tare_samples = max(1, n_samples)
        self._tare_req.set()

    def _run(self) -> None:
        t0 = time.monotonic()
        rate_t0 = t0
        rate_n = 0
        prev_seq: int | None = None
        tare_rem = 0
        tare_accum = [0.0] * 8

        while not self._stop.is_set():
            try:
                sample = self.dev.read_sample()
            except TimeoutError:
                continue

            if prev_seq is not None and ((prev_seq + 1) & 0xFF) != sample.seq:
                self.dropped += (sample.seq - prev_seq - 1) & 0xFF
            prev_seq = sample.seq

            if self._tare_req.is_set():
                self._tare_req.clear()
                tare_rem = self._tare_samples
                tare_accum = [0.0] * 8

            if tare_rem > 0:
                for i, v in enumerate(sample.raw):
                    tare_accum[i] += v
                tare_rem -= 1
                if tare_rem == 0:
                    self.dev.baseline = [s / self._tare_samples for s in tare_accum]
                fm = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            else:
                fm = sample.forces_moments or (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            now = time.monotonic()
            with self.lock:
                self.t_buf.append(now - t0)
                self.fm_buf.append(fm)
                self.sample_count += 1

            rate_n += 1
            if now - rate_t0 >= 0.5:
                self.measured_hz = rate_n / (now - rate_t0)
                rate_t0 = now
                rate_n = 0

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, int, int, float]:
        with self.lock:
            if not self.t_buf:
                return np.empty(0), np.empty((0, 6)), self.sample_count, self.dropped, self.measured_hz
            t_arr = np.fromiter(self.t_buf, dtype=np.float64, count=len(self.t_buf))
            fm_arr = np.array(self.fm_buf, dtype=np.float64)
            return t_arr, fm_arr, self.sample_count, self.dropped, self.measured_hz


def main() -> int:
    parser = argparse.ArgumentParser(description="Live 1000 Hz plot for Bertec AM6817-E")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--window", type=float, default=5.0, help="Scrolling window in seconds")
    parser.add_argument("--fps", type=float, default=30.0, help="Plot refresh rate in Hz")
    parser.add_argument("--tare", type=int, default=200, help="Initial tare samples (0 to skip)")
    args = parser.parse_args()

    max_samples = max(100, int(args.window * 1050))

    with BertecAM6817(args.port) as dev:
        info = dev.get_info()
        cal = dev.read_calibration()
        print(f"Connected: {cal.model} ({info.serial}) @ {info.sample_rate_hz} Hz")
        if args.tare > 0:
            print(f"Taring ({args.tare} samples)...")
            dev.tare(args.tare)

        reader = LivePlotReader(dev, max_samples=max_samples)
        reader.start()

        fig, (ax_f, ax_m) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
        fig.suptitle(f"Bertec {cal.model} ({info.serial}) — press 't' to tare")

        f_labels = ["Fx", "Fy", "Fz"]
        m_labels = ["Mx", "My", "Mz"]
        f_lines = [ax_f.plot([], [], lw=1.2, label=lbl)[0] for lbl in f_labels]
        m_lines = [ax_m.plot([], [], lw=1.2, label=lbl)[0] for lbl in m_labels]

        ax_f.set_ylabel("Force [N]")
        ax_f.grid(True, alpha=0.3)
        ax_f.legend(loc="upper left", ncol=3)
        ax_f.set_ylim(-50, 250)

        ax_m.set_ylabel("Moment [Nm]")
        ax_m.set_xlabel("Time [s]")
        ax_m.grid(True, alpha=0.3)
        ax_m.legend(loc="upper left", ncol=3)
        ax_m.set_ylim(-50, 50)

        status_text = ax_f.text(
            0.99, 0.94, "", transform=ax_f.transAxes, ha="right", va="top", fontsize=9
        )

        def on_key(event) -> None:
            if event.key == "t":
                reader.request_tare(max(1, args.tare))

        fig.canvas.mpl_connect("key_press_event", on_key)

        def update(_frame):
            t_arr, fm_arr, count, dropped, hz = reader.snapshot()
            if t_arr.size < 2:
                return (*f_lines, *m_lines, status_text)

            t_end = t_arr[-1]
            t_start = max(0.0, t_end - args.window)
            ax_f.set_xlim(t_start, t_start + args.window)

            for i in range(3):
                f_lines[i].set_data(t_arr, fm_arr[:, i])
                m_lines[i].set_data(t_arr, fm_arr[:, i + 3])

            # Auto-expand y-limits if signal exceeds current view
            f_min, f_max = float(fm_arr[:, :3].min()), float(fm_arr[:, :3].max())
            cur_f_lo, cur_f_hi = ax_f.get_ylim()
            if f_min < cur_f_lo or f_max > cur_f_hi:
                pad = max(10.0, 0.1 * (f_max - f_min))
                ax_f.set_ylim(min(cur_f_lo, f_min - pad), max(cur_f_hi, f_max + pad))

            m_min, m_max = float(fm_arr[:, 3:].min()), float(fm_arr[:, 3:].max())
            cur_m_lo, cur_m_hi = ax_m.get_ylim()
            if m_min < cur_m_lo or m_max > cur_m_hi:
                pad = max(5.0, 0.1 * (m_max - m_min))
                ax_m.set_ylim(min(cur_m_lo, m_min - pad), max(cur_m_hi, m_max + pad))

            status_text.set_text(f"{hz:6.1f} Hz | n={count} | drop={dropped}")
            return (*f_lines, *m_lines, status_text)

        interval_ms = max(1, int(round(1000.0 / args.fps)))
        _anim = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)

        try:
            plt.tight_layout()
            plt.show()
        finally:
            reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
