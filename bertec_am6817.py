#!/usr/bin/env python3
"""
Direct Linux USB-serial driver for the Bertec AM6817-E digital amplifier.

Uses standard POSIX termios (no pyserial dependency) at 1,500,000 baud 8N1.
"""

from __future__ import annotations

import argparse
import csv
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass
from typing import Iterator, Optional

BAUD_RATE = termios.B1500000
SYNC_BYTE = 0x7E
ESC_BYTE = 0x7D
ESC_XOR = 0x20
PAYLOAD_SIZE = 17  # 1 seq + 8 * uint16 LE

CMD_STOP_PING = b"\x01"
CMD_GET_INFO = b"\x02"
CMD_GET_CALIBRATION = b"\x03"
CMD_START_STREAM = b"\x06\x00"


@dataclass
class DeviceInfo:
    """Metadata returned by the 0x02 device info command."""

    firmware_raw: int
    eeprom_size: int
    sample_rate_hz: int
    num_channels: int
    serial: str


@dataclass
class CalibrationData:
    """Parsed EEPROM calibration block returned by the 0x03 command."""

    serial: str
    model: str
    dimensions_mm: tuple[float, ...]
    ranges: tuple[float, ...]
    matrix: list[list[float]]  # 6x8: Fx, Fy, Fz, Mx, My, Mz


@dataclass
class Sample:
    """Single 1 kHz force-plate sample."""

    seq: int
    raw: tuple[int, ...]
    forces_moments: Optional[tuple[float, float, float, float, float, float]] = None


class BertecAM6817:
    """POSIX serial interface to a Bertec AM6817-E amplifier over /dev/ttyUSB*."""

    def __init__(self, port: str = "/dev/ttyUSB0", timeout: float = 0.5) -> None:
        self.port = port
        self.timeout = timeout
        self._fd: Optional[int] = None
        self._rx_buf = bytearray()
        self.calibration: Optional[CalibrationData] = None
        self.baseline: Optional[list[float]] = None

    def __enter__(self) -> BertecAM6817:
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._fd is not None:
            return
        port = self.port
        by_id = "/dev/serial/by-id/usb-Bertec_Corporation_AM6817-E-if00-port0"
        if not os.path.exists(port) and os.path.exists(by_id):
            port = os.path.realpath(by_id)
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0  # iflag: raw
        attrs[1] = 0  # oflag: raw
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0  # lflag: raw
        attrs[4] = BAUD_RATE
        attrs[5] = BAUD_RATE
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        self._fd = fd
        time.sleep(0.05)
        self.stop_stream()

    def close(self) -> None:
        if self._fd is None:
            return
        try:
            self.stop_stream()
        except OSError:
            pass
        os.close(self._fd)
        self._fd = None
        self._rx_buf.clear()

    def _require_fd(self) -> int:
        if self._fd is None:
            raise RuntimeError("Device is not open")
        return self._fd

    def _read_until_idle(self, idle_timeout: float = 0.15, max_bytes: int = 8192) -> bytes:
        fd = self._require_fd()
        out = bytearray()
        while len(out) < max_bytes:
            r, _, _ = select.select([fd], [], [], idle_timeout)
            if not r:
                break
            chunk = os.read(fd, max_bytes - len(out))
            if not chunk:
                break
            out.extend(chunk)
        return bytes(out)

    def _read_exact(self, n: int, timeout: float = 0.5) -> bytes:
        fd = self._require_fd()
        deadline = time.monotonic() + timeout
        while len(self._rx_buf) < n:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"Timed out waiting for {n} bytes (got {len(self._rx_buf)})")
            r, _, _ = select.select([fd], [], [], remaining)
            if not r:
                continue
            chunk = os.read(fd, 4096)
            if chunk:
                self._rx_buf.extend(chunk)
        data = bytes(self._rx_buf[:n])
        del self._rx_buf[:n]
        return data

    def stop_stream(self) -> None:
        fd = self._require_fd()
        os.write(fd, CMD_STOP_PING)
        time.sleep(0.02)
        termios.tcflush(fd, termios.TCIFLUSH)
        self._rx_buf.clear()

    def get_info(self) -> DeviceInfo:
        fd = self._require_fd()
        self.stop_stream()
        os.write(fd, CMD_GET_INFO)
        resp = self._read_exact(24, timeout=self.timeout)
        if resp[0] != 0x01:
            raise RuntimeError(f"Unexpected ACK byte for get_info: 0x{resp[0]:02x}")
        fw_raw, eeprom_size, sample_rate, num_ch = struct.unpack("<HHHB", resp[1:8])
        serial = resp[8:24].rstrip(b"\x00").decode("ascii", errors="replace")
        return DeviceInfo(
            firmware_raw=fw_raw,
            eeprom_size=eeprom_size,
            sample_rate_hz=sample_rate,
            num_channels=num_ch,
            serial=serial,
        )

    def read_calibration(self) -> CalibrationData:
        fd = self._require_fd()
        self.stop_stream()
        os.write(fd, CMD_GET_CALIBRATION)
        resp = self._read_exact(1025, timeout=1.0)
        if resp[0] != 0x01:
            raise RuntimeError(f"Unexpected ACK byte for read_calibration: 0x{resp[0]:02x}")
        text = resp[1:].split(b"\x00", 1)[0].decode("ascii", errors="replace")
        cal = _parse_calibration_text(text)
        self.calibration = cal
        return cal

    def start_stream(self) -> None:
        fd = self._require_fd()
        self.stop_stream()
        os.write(fd, CMD_START_STREAM)
        ack = self._read_exact(1, timeout=self.timeout)
        if ack[0] != 0x01:
            raise RuntimeError(f"Failed to start stream, ACK=0x{ack[0]:02x}")

    def read_sample(self) -> Sample:
        """Read and unescape one 0x7E-delimited sample frame from the active stream."""
        fd = self._require_fd()
        while True:
            # Ensure we have a starting SYNC_BYTE and a subsequent SYNC_BYTE
            start = self._rx_buf.find(SYNC_BYTE)
            end = self._rx_buf.find(SYNC_BYTE, start + 1) if start != -1 else -1

            if start == -1 or end == -1:
                if start > 0:
                    del self._rx_buf[:start]
                r, _, _ = select.select([fd], [], [], self.timeout)
                if not r:
                    raise TimeoutError("Timed out waiting for stream frame")
                chunk = os.read(fd, 4096)
                if chunk:
                    self._rx_buf.extend(chunk)
                continue

            raw_frame = bytes(self._rx_buf[start + 1 : end])
            # Keep the ending SYNC_BYTE as the start of the next frame
            del self._rx_buf[:end]

            payload = _unescape_hdlc(raw_frame)
            if len(payload) != PAYLOAD_SIZE:
                continue

            seq = payload[0]
            raw = struct.unpack("<8H", payload[1:])
            fm = self._convert_raw(raw)
            return Sample(seq=seq, raw=raw, forces_moments=fm)

    def _convert_raw(
        self, raw: tuple[int, ...]
    ) -> Optional[tuple[float, float, float, float, float, float]]:
        if self.calibration is None:
            return None
        base = self.baseline if self.baseline is not None else [32768.0] * 8
        delta = [raw[i] - base[i] for i in range(8)]
        out = [sum(row[j] * delta[j] for j in range(8)) for row in self.calibration.matrix]
        return (out[0], out[1], out[2], out[3], out[4], out[5])

    def tare(self, n_samples: int = 200) -> list[float]:
        """Average n_samples while unloaded to set the software zero baseline."""
        self.start_stream()
        sums = [0.0] * 8
        try:
            for _ in range(n_samples):
                sample = self.read_sample()
                for i, val in enumerate(sample.raw):
                    sums[i] += val
        finally:
            self.stop_stream()
        self.baseline = [s / n_samples for s in sums]
        return self.baseline

    def stream(self, duration_s: Optional[float] = None) -> Iterator[Sample]:
        """Yield samples until duration_s elapses (or indefinitely if None)."""
        self.start_stream()
        t0 = time.monotonic()
        try:
            while duration_s is None or (time.monotonic() - t0) < duration_s:
                yield self.read_sample()
        finally:
            self.stop_stream()


def _unescape_hdlc(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        b = data[i]
        if b == ESC_BYTE:
            if i + 1 >= n:
                return b""
            out.append(data[i + 1] ^ ESC_XOR)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


def _parse_calibration_text(text: str) -> CalibrationData:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    serial = lines[0] if lines else ""
    matrix: list[list[float]] = []
    ranges: tuple[float, ...] = ()
    model = ""
    dims: tuple[float, ...] = ()
    section = ""

    for line in lines[1:]:
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].lower()
            continue
        if section == "calibration matrix":
            parts = line.split()
            if len(parts) == 9:
                matrix.append([float(x) for x in parts[1:]])
        elif section == "ranges":
            ranges = tuple(float(x) for x in line.split())
        elif section == "nameplate":
            if line.startswith("M "):
                model = line[2:].strip()
            elif line.startswith("G "):
                dims = tuple(float(x) for x in line[2:].split())

    if len(matrix) != 6:
        raise ValueError(f"Expected 6 calibration rows, got {len(matrix)}")

    return CalibrationData(
        serial=serial,
        model=model,
        dimensions_mm=dims,
        ranges=ranges,
        matrix=matrix,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Read Bertec AM6817-E force plate over USB")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial device path")
    parser.add_argument("--info", action="store_true", help="Print device info and calibration")
    parser.add_argument("--tare", type=int, default=200, help="Samples to average for tare (0 to skip)")
    parser.add_argument("--duration", type=float, default=5.0, help="Stream duration in seconds (0 = forever)")
    parser.add_argument("--print-hz", type=float, default=10.0, help="Console print rate in Hz")
    parser.add_argument("--csv", type=str, default=None, help="Optional CSV output path")
    args = parser.parse_args()

    with BertecAM6817(args.port) as dev:
        info = dev.get_info()
        cal = dev.read_calibration()

        if args.info:
            print(f"Model:       {cal.model}")
            print(f"Serial:      {info.serial}")
            print(f"Sample rate: {info.sample_rate_hz} Hz ({info.num_channels} raw ADC channels)")
            print(f"Ranges:      {cal.ranges}")
            print(f"Dimensions:  {cal.dimensions_mm}")
            return 0

        if args.tare > 0:
            print(f"Taring ({args.tare} samples)...", file=sys.stderr)
            dev.tare(args.tare)

        duration = None if args.duration <= 0 else args.duration
        print_interval = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
        last_print = 0.0
        prev_seq: Optional[int] = None
        dropped = 0
        count = 0

        csv_file = open(args.csv, "w", newline="") if args.csv else None
        writer = csv.writer(csv_file) if csv_file else None
        if writer:
            writer.writerow(
                ["sample", "seq", "Fx_N", "Fy_N", "Fz_N", "Mx_Nm", "My_Nm", "Mz_Nm"]
                + [f"ch{i}" for i in range(8)]
            )

        try:
            for sample in dev.stream(duration_s=duration):
                if prev_seq is not None and ((prev_seq + 1) & 0xFF) != sample.seq:
                    dropped += (sample.seq - prev_seq - 1) & 0xFF
                prev_seq = sample.seq
                count += 1

                fx, fy, fz, mx, my, mz = sample.forces_moments or (0.0,) * 6
                if writer:
                    writer.writerow([count, sample.seq, fx, fy, fz, mx, my, mz, *sample.raw])

                now = time.monotonic()
                if print_interval > 0 and (now - last_print) >= print_interval:
                    last_print = now
                    print(
                        f"n={count:6d} drop={dropped:2d} | "
                        f"Fx={fx:8.2f} Fy={fy:8.2f} Fz={fz:8.2f} N | "
                        f"Mx={mx:7.2f} My={my:7.2f} Mz={mz:7.2f} Nm"
                    )
        except KeyboardInterrupt:
            pass
        finally:
            if csv_file:
                csv_file.close()

        print(f"Done: {count} samples, {dropped} dropped frames.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
