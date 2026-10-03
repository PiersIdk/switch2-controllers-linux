"""Joy-Con 2 optical mouse, switched on automatically like a Switch 2.

Each Joy-Con 2 has an optical sensor on its rail. With the mouse feature
enabled (protocol.FEATURE_MOUSE) every input report carries:

  * bytes 16-17 / 18-19 - cumulative X / Y motion counters (u16, wrapping)
  * byte 21             - 0x11 when nothing is in front of the sensor
  * byte 23             - 0x00 while the sensor sees a surface, 0x0C when it
                          does not (a finger on the rail reads 0x05-0x08)

(Verified from raw captures: byte 23 is 0x00 in every report taken while the
Joy-Con slid on a desk and in none taken in hand or in the air.) Reports sent
before the sensor is initialised are all-zero, so a surface also requires a
non-zero byte 21.

A Joy-Con that has seen a surface for SURFACE_ON_S becomes a mouse; lifting
it for SURFACE_OFF_S turns it back into a controller. While it is a mouse,
its shoulder buttons click, its stick click is the middle button and its
stick scrolls; the bridge keeps everything else on the gamepad.
"""

from __future__ import annotations

import logging
from typing import Optional

from evdev import UInput, ecodes as e

from . import protocol as P

logger = logging.getLogger(__name__)

SURFACE_ON_S = 1.0
SURFACE_OFF_S = 0.4

# Optical counts -> pixels. The sensor is high resolution: a brisk sweep in
# the captures ran ~20000 counts/s, so 1:1 would throw the pointer across the
# screen. Reports arrive at ~135 Hz on a 7.5 ms link, so deltas are emitted
# directly rather than resampled; fractions carry over between reports so
# slow, precise moves aren't rounded away.
SENSITIVITY = 0.15
DEADZONE = 2           # raw counts of jitter ignored per report
MAX_STEP = 200         # pixels per report

# Stick scroll (calibrated stick, -1..1).
SCROLL_DEADZONE = 0.12
SCROLL_MAX_LINES_PER_S = 12.0  # full stick; was 20 (too fast in use)
SCROLL_CURVE = 2.0             # gentler near the centre for fine scrolling
SCROLL_MAX_STEP = 3


def on_surface(raw: bytes) -> bool:
    return len(raw) > 23 and raw[23] == 0x00 and raw[21] != 0x00


def _u16(raw: bytes, i: int) -> int:
    return raw[i] | (raw[i + 1] << 8)


def _delta_u16(curr: int, prev: int) -> int:
    d = (curr - prev) & 0xFFFF
    return d - 0x10000 if d > 0x7FFF else d


class SurfaceSwitch:
    """Debounces the on-surface flag into a mouse / controller state."""

    def __init__(self) -> None:
        self.active = False
        self._since: Optional[float] = None  # when the flag last changed

    def update(self, surface: bool, now: float) -> bool:
        """Feed one report; returns True on the report the state flips."""
        if surface == self.active:
            self._since = None
            return False
        if self._since is None:
            self._since = now
            return False
        if now - self._since >= (SURFACE_ON_S if surface else SURFACE_OFF_S):
            self.active = surface
            self._since = None
            return True
        return False


class OpticalMouse:
    """Virtual mouse fed by one Joy-Con's optical counters, clicks and stick."""

    def __init__(self, name: str) -> None:
        self.ui = UInput(
            {
                e.EV_REL: [e.REL_X, e.REL_Y, e.REL_WHEEL, e.REL_WHEEL_HI_RES],
                e.EV_KEY: [e.BTN_LEFT, e.BTN_RIGHT, e.BTN_MIDDLE],
            },
            name=name,
            vendor=P.NINTENDO_VENDOR_ID,
            version=0x0100,
            bustype=e.BUS_BLUETOOTH,
        )
        logger.info("created virtual mouse: %s", name)
        self._prev: Optional[tuple[int, int]] = None
        self._frac = [0.0, 0.0]
        self._buttons = {e.BTN_LEFT: 0, e.BTN_RIGHT: 0, e.BTN_MIDDLE: 0}
        self._wheel = 0.0
        self._wheel_hires = 0.0
        self._last_at: Optional[float] = None

    def reset_motion(self) -> None:
        """Forget the last counter reading so re-entering mouse mode can't jump."""
        self._prev = None
        self._frac = [0.0, 0.0]
        self._wheel = 0.0
        self._wheel_hires = 0.0
        self._last_at = None

    def update(self, raw: bytes, left: bool, right: bool, middle: bool,
               scroll_y: float, now: float) -> None:
        changed = False

        pos = (_u16(raw, 16), _u16(raw, 18))
        if self._prev is not None:
            for axis, code in ((0, e.REL_X), (1, e.REL_Y)):
                raw_delta = _delta_u16(pos[axis], self._prev[axis])
                if abs(raw_delta) <= DEADZONE:
                    continue
                self._frac[axis] += raw_delta * SENSITIVITY
                step = max(-MAX_STEP, min(MAX_STEP, int(self._frac[axis])))
                if step:
                    self._frac[axis] -= step
                    self.ui.write(e.EV_REL, code, step)
                    changed = True
        self._prev = pos

        for code, pressed in ((e.BTN_LEFT, left), (e.BTN_RIGHT, right), (e.BTN_MIDDLE, middle)):
            value = 1 if pressed else 0
            if self._buttons[code] != value:
                self.ui.write(e.EV_KEY, code, value)
                self._buttons[code] = value
                changed = True

        changed |= self._scroll(scroll_y, now)
        if changed:
            self.ui.syn()

    def _scroll(self, y: float, now: float) -> bool:
        dt = 0.0 if self._last_at is None else min(0.1, now - self._last_at)
        self._last_at = now
        if abs(y) <= SCROLL_DEADZONE or dt <= 0.0:
            return False
        norm = min(1.0, (abs(y) - SCROLL_DEADZONE) / (1.0 - SCROLL_DEADZONE))
        lines = (1.0 if y > 0 else -1.0) * (norm ** SCROLL_CURVE) * SCROLL_MAX_LINES_PER_S * dt
        # Two accumulators over the same motion: hi-res readers get 1/120ths
        # of a line, legacy REL_WHEEL readers whole lines.
        self._wheel_hires += lines * 120.0
        self._wheel += lines
        wrote = False
        hires = max(-SCROLL_MAX_STEP * 120, min(SCROLL_MAX_STEP * 120, int(self._wheel_hires)))
        if hires:
            self.ui.write(e.EV_REL, e.REL_WHEEL_HI_RES, hires)
            self._wheel_hires -= hires
            wrote = True
        step = max(-SCROLL_MAX_STEP, min(SCROLL_MAX_STEP, int(self._wheel)))
        if step:
            self.ui.write(e.EV_REL, e.REL_WHEEL, step)
            self._wheel -= step
            wrote = True
        return wrote

    def release(self) -> None:
        changed = False
        for code in self._buttons:
            if self._buttons[code]:
                self.ui.write(e.EV_KEY, code, 0)
                self._buttons[code] = 0
                changed = True
        if changed:
            self.ui.syn()
        self.reset_motion()

    def close(self) -> None:
        try:
            self.release()
            self.ui.close()
        except Exception:  # noqa: BLE001
            pass
