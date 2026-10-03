"""Gyro zero-rate offset: measured whenever a controller lies still, then
subtracted from every report.

Every gyro reads a small non-zero rate at rest - ~0.1-0.25 deg/s per axis on
a Joy-Con 2 measured flat on a desk - which emulators integrate into a slow,
constant spin. Any ~1 s window where the gyro barely moves and the
accelerometer is steady (so the controller isn't turning) is taken as "at
rest", and its mean gyro reading becomes the offset. Offsets are kept per
controller MAC in imu-bias.json so they apply from the first report after a
reconnect.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections import deque
from typing import Optional

from .config import CONFIG_DIR

logger = logging.getLogger(__name__)

BIAS_PATH = CONFIG_DIR / "imu-bias.json"

WINDOW = 135                # reports, ~1 s at the 7.5 ms link interval
GYRO_STILL_RANGE = 15       # raw LSB (~0.9 deg/s) max-min per axis; rest noise is ~1 LSB
ACCEL_STILL_RANGE = 80      # raw LSB (~0.02 g) max-min per axis
SAVE_MIN_INTERVAL_S = 30.0
SAVE_MIN_CHANGE = 2.0       # raw LSB

_file_lock = threading.Lock()


def _load_all() -> dict:
    try:
        return json.loads(BIAS_PATH.read_text())
    except (OSError, ValueError):
        return {}


class GyroBias:
    def __init__(self, mac: str) -> None:
        self.mac = mac.upper()
        saved = _load_all().get(self.mac)
        self.bias: tuple[float, float, float] = tuple(saved) if saved else (0.0, 0.0, 0.0)
        self._saved = self.bias
        self._saved_at = 0.0
        self._gyro: deque = deque(maxlen=WINDOW)
        self._accel: deque = deque(maxlen=WINDOW)

    def correct(self, accel: tuple, gyro: tuple) -> tuple[float, float, float]:
        """Feed one raw sample; returns the offset-corrected gyro."""
        self._gyro.append(gyro)
        self._accel.append(accel)
        if len(self._gyro) == WINDOW and self._still():
            self.bias = tuple(sum(g[i] for g in self._gyro) / WINDOW for i in range(3))
            self._gyro.clear()
            self._accel.clear()
            self._maybe_save()
        return tuple(gyro[i] - self.bias[i] for i in range(3))

    def _still(self) -> bool:
        for samples, limit in ((self._gyro, GYRO_STILL_RANGE), (self._accel, ACCEL_STILL_RANGE)):
            for i in range(3):
                axis = [s[i] for s in samples]
                if max(axis) - min(axis) > limit:
                    return False
        return True

    def _maybe_save(self) -> None:
        now = time.monotonic()
        changed = max(abs(self.bias[i] - self._saved[i]) for i in range(3))
        if changed < SAVE_MIN_CHANGE or now - self._saved_at < SAVE_MIN_INTERVAL_S:
            return
        with _file_lock:
            data = _load_all()
            data[self.mac] = [round(b, 2) for b in self.bias]
            try:
                CONFIG_DIR.mkdir(parents=True, exist_ok=True)
                tmp = BIAS_PATH.with_suffix(".json.tmp")
                tmp.write_text(json.dumps(data, indent=2))
                tmp.replace(BIAS_PATH)
            except OSError as exc:
                logger.debug("saving gyro offset failed: %s", exc)
                return
        self._saved, self._saved_at = self.bias, now
        logger.info("gyro offset for %s calibrated: %s (raw LSB)", self.mac,
                    [round(b, 1) for b in self.bias])
