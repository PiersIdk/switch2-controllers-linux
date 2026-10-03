"""Joy-Con 2 pairing: merge a Left and a Right Joy-Con into one virtual pad.

Each Joy-Con is still its own BLE link with its own bridge worker (connect,
handshake, idle sleep, reconnect all unchanged). Instead of each worker
owning a gamepad, Joy-Con workers attach to the bridge's JoyConPair, which
owns a single gamepad + motion device + DSU slot and composes every frame
from the latest state of both halves:

  * buttons      - each half masked to the bits it physically drives, OR'd
  * left stick   - from the Left Joy-Con, right stick from the Right
  * ZL / ZR      - digital, full-scale trigger axes
  * motion / DSU - from the Right Joy-Con (Left if only it is connected); the
                   Left's own motion also goes out on a second DSU slot, so an
                   emulator's dual-Joy-Con mode can use both (see _sync_left_dsu)
  * rumble       - fanned out to both halves

With only one half connected it is its own sideways controller, like a lone
Joy-Con on a Switch: the stick is rotated, SL/SR become the shoulders, and
each button acts as the full-pad button in the same position. It gets its
own device ("Joy-Con 2 (L)" / "(R)"), swapped for "Joy-Con 2 Pair" when the
partner connects.

Either half turns into a mouse when its optical sensor sits on a surface
(see mouse.py); while it does, its clicks and stick go to its own virtual
mouse and the rest of its buttons stay on the pad.

The pair is presented as a combined Joy-Con pad (see JOYCON_PAIR_PID) and a
lone half as its own Joy-Con, so Steam / SDL see a standard layout. When both halves sleep, the last pad stays alive (like a single
controller's pad does) until one comes back or the bridge shuts down.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from . import protocol as P
from .gamepad import SwitchGamepad
from .motion_evdev import MotionEvdev
from .imu_frame import sideways as _sideways_imu, to_dsu as _dsu_imu, upright as _upright_imu
from .mouse import OpticalMouse, SurfaceSwitch, on_surface

if TYPE_CHECKING:
    from .device import SwitchController
    from .dsu import DSUServer

logger = logging.getLogger(__name__)

LEFT, RIGHT = "left", "right"

_B = P.SWITCH_BUTTONS

# Device version per layout. Steam keys a controller's saved name and bindings
# on its SDL GUID (bus, vendor, product, version) - not its name - and every
# pad here reports the Pro Controller 2 product ID, so each layout needs its
# own version or Steam treats the pair, each lone Joy-Con and a real Pro 2
# (0x0100) as one controller.
_LAYOUT_VERSIONS = {"pair": 0x0101, "left": 0x0102, "right": 0x0103}
# Steam's own controller identity (the handle it files a controller's name
# under) ignores the version and the device name and, with uinput giving no
# way to set a serial, is the same for every 057e:2069 pad. So a lone Joy-Con
# reports its real product ID to get an identity of its own.
#
# The pair reports 0x2008, the ID Linux's joycond gives a combined L+R
# Joy-Con pad, which SDL/Steam know as a Joy-Con pair (Joy-Con glyphs), and
# which keeps it from sharing an identity with a real Pro Controller 2.
JOYCON_PAIR_PID = 0x2008
_LAYOUT_PRODUCTS = {
    "pair": JOYCON_PAIR_PID,
    "left": P.JOYCON2_LEFT_PID,
    "right": P.JOYCON2_RIGHT_PID,
}

# Per side: the bits this half drives, and (left click, right click, middle
# click) while it is a mouse.
_SIDE_BUTTONS = {LEFT: P.JOYCON_LEFT_BUTTONS, RIGHT: P.JOYCON_RIGHT_BUTTONS}
_MOUSE_CLICKS = {
    LEFT: (_B["L"], _B["ZL"], _B["L_STK"]),
    RIGHT: (_B["R"], _B["ZR"], _B["R_STK"]),
}

# Sideways (single Joy-Con, rail up) -> the full-pad button sitting in the
# same position, so the pad's own button map (and Steam's bindings for it)
# apply unchanged. Full pad positions: A right, B bottom, X top, Y left.
# The rail-side shoulders (R/ZR, L/ZL) are left unmapped held sideways.
#   Right Joy-Con rotated clockwise:    Y top, X right, A bottom, B left
#   Left Joy-Con rotated anticlockwise: d-pad Right top, Down right,
#                                       Left bottom, Up left
_SIDEWAYS = {
    RIGHT: {
        "Y": "X", "X": "A", "A": "B", "B": "Y",
        "SL_R": "L", "SR_R": "R",
        "PLUS": "PLUS", "HOME": "HOME", "C": "C", "R_STK": "L_STK",
    },
    LEFT: {
        "RIGHT": "X", "DOWN": "A", "LEFT": "B", "UP": "Y",
        "SL_L": "L", "SR_L": "R",
        # No Home button on a Left Joy-Con; held alone, its Capture is Home.
        "MINUS": "PLUS", "CAPTURE": "HOME", "L_STK": "L_STK",
    },
}


def side_of(product_id: int) -> Optional[str]:
    if product_id == P.JOYCON2_LEFT_PID:
        return LEFT
    if product_id == P.JOYCON2_RIGHT_PID:
        return RIGHT
    return None


def _sideways(side: str, buttons: int) -> int:
    out = 0
    for src, dst in _SIDEWAYS[side].items():
        if buttons & _B[src]:
            out |= _B[dst]
    return out


def _rotate_sideways(side: str, stick: tuple[float, float]) -> tuple[float, float]:
    """Upright stick (x right, y up) -> what it means held sideways."""
    x, y = stick
    if side == RIGHT:  # rotated clockwise: device-up points right
        return y, -x
    return -y, x       # Left, anticlockwise: device-up points left


@dataclass
class _Half:
    ctrl: "SwitchController"
    side: str
    buttons: int = 0
    stick: tuple[float, float] = (0.0, 0.0)
    surface: SurfaceSwitch = field(default_factory=SurfaceSwitch)


class JoyConPair:
    def __init__(self, button_map: dict, dsu: Optional["DSUServer"] = None,
                 *, sideways: bool = True, mouse: bool = True,
                 extra_dsu_slots=None):
        # Assigned by the bridge when the first half connects (see
        # Bridge.claim_player) and released once both halves are gone.
        self.player: Optional[int] = None
        self.slot = 0
        self.button_map = button_map
        self.dsu = dsu
        self.sideways = sideways
        self.mouse_enabled = mouse
        self.gamepad: Optional[SwitchGamepad] = None
        self.motion: Optional[MotionEvdev] = None
        # Which layout the current pad was created for: "pair", or LEFT /
        # RIGHT for a lone sideways Joy-Con (see _ensure_device).
        self._device_layout: Optional[str] = None
        # One virtual mouse per side, created the first time it's needed and
        # kept (like the pad) so reconnects don't churn input devices.
        self._mice: dict[str, OpticalMouse] = {}
        self._halves: dict[str, _Half] = {}
        self._lock = threading.Lock()
        # (claim, release) for the Left half's own DSU slot while paired.
        # The pad has one SDL motion device and the pair's DSU slot carries the
        # Right's motion, so without this the Left's motion is unreachable.
        self._extra_dsu_slots = extra_dsu_slots
        self._left_dsu_slot: Optional[int] = None
        # When the last half left, while the pad is still kept around.
        self._empty_since: Optional[float] = None

    def set_player(self, player: int) -> None:
        self.player = player
        self.slot = max(0, min(3, player - 1))

    def is_empty(self) -> bool:
        with self._lock:
            return not self._halves

    def attach(self, ctrl: "SwitchController", relayout: bool = True) -> None:
        """``relayout=False`` defers creating/replacing the pad to a later
        sync_device(), so moving both halves at once (Bridge.set_joycon_split)
        doesn't flash a single-Joy-Con device in between."""
        side = side_of(ctrl.product_id)
        if side is None:
            raise ValueError(f"{ctrl.mac} is not a Joy-Con 2")
        with self._lock:
            self._halves[side] = _Half(ctrl, side)
            self._empty_since = None
            if relayout:
                self._ensure_device()
                # Re-hook every attach: detaching the last half unhooks
                # rumble, and a reused pad (same layout) isn't recreated.
                self.gamepad.rumble_cb = self._on_rumble
            if self.dsu is not None:
                # The pair's slot is identified by its motion source half.
                src = self._halves[self._motion_side()].ctrl
                self.dsu.set_slot(self.slot, True, mac=src.mac, battery_mv=src.battery_mv or 0)
            self._sync_left_dsu()
            self._emit()
            sides = "+".join(sorted(self._halves))
        logger.info("Joy-Con %s attached to P%s pair (now %s)", side, self.player, sides)

    def sync_device(self) -> None:
        """Bring the pad in line with the halves now attached (see attach)."""
        with self._lock:
            if not self._halves:
                return
            self._ensure_device()
            self.gamepad.rumble_cb = self._on_rumble
            self._emit()

    def detach(self, ctrl: "SwitchController", relayout: bool = True) -> None:
        side = side_of(ctrl.product_id)
        with self._lock:
            half = self._halves.get(side)
            if half is None or half.ctrl is not ctrl:
                return
            del self._halves[side]
            mouse = self._mice.get(side)
            if mouse is not None:
                mouse.release()
            self._sync_left_dsu()
            if self._halves:
                if relayout:
                    self._ensure_device()
                    self._emit()
            else:
                if self.gamepad is not None:
                    self.gamepad.rumble_cb = None
                    self.gamepad.release_all()
                if self.dsu is not None:
                    self.dsu.set_slot(self.slot, False)
                self._empty_since = time.monotonic()
        logger.info("Joy-Con %s detached from P%s pair", side, self.player)

    def on_input(self, ctrl: "SwitchController", report: P.InputReport) -> None:
        side = side_of(ctrl.product_id)
        (lx, ly), (rx, ry), _lt, _rt = ctrl.calibrated_input(report)
        now = time.monotonic()
        with self._lock:
            half = self._halves.get(side)
            if half is None or half.ctrl is not ctrl:
                return
            half.buttons = report.buttons & _SIDE_BUTTONS[side]
            half.stick = (lx, ly) if side == LEFT else (rx, ry)
            if self.mouse_enabled:
                self._update_mouse(half, report.raw, now)
            self._emit()
            # Into the emulator frame for how the half is held: upright as one
            # side of a pair (or alone with sideways mode off), else sideways.
            layout = self._layout()
            imu = _upright_imu(report) if layout == "pair" else _sideways_imu(side, report)
            dsu_imu = _dsu_imu(imu)
            if side == self._motion_side():
                if self.motion is not None:
                    self.motion.update(imu)
                if self.dsu is not None:
                    self._update_dsu(dsu_imu)
            elif side == LEFT and self._left_dsu_slot is not None and self.dsu is not None:
                self._update_left_dsu(dsu_imu, half)

    def remove_stale_pad(self, now: float, grace_s: float) -> None:
        """Drop the kept pad (and mice) once both halves have been gone for
        grace_s; the next attach recreates them (see Bridge's
        PAD_REMOVE_GRACE_S)."""
        with self._lock:
            if self._halves or self._empty_since is None or now - self._empty_since < grace_s:
                return
            self._empty_since = None
            if self.gamepad is not None:
                self.gamepad.close()
                self.gamepad = None
                logger.info("removed Joy-Con virtual pad (both halves gone %.0fs)", grace_s)
            if self.motion is not None:
                self.motion.close()
                self.motion = None
            self._device_layout = None
            for mouse in self._mice.values():
                mouse.close()
            self._mice.clear()

    def close(self) -> None:
        with self._lock:
            self._halves.clear()
            if self.gamepad is not None:
                self.gamepad.rumble_cb = None
                self.gamepad.close()
                self.gamepad = None
            if self.motion is not None:
                self.motion.close()
                self.motion = None
            self._device_layout = None
            for mouse in self._mice.values():
                mouse.close()
            self._mice.clear()
            self._sync_left_dsu()
            if self.dsu is not None:
                self.dsu.set_slot(self.slot, False)

    # ------------------------------------------------------------------ #

    def _layout(self) -> str:
        if len(self._halves) == 1 and self.sideways:
            (side,) = self._halves
            return side
        return "pair"

    def _ensure_device(self) -> None:
        """Give the connected halves a pad named for what they are: one
        sideways Joy-Con is its own controller, two are a pair. Swapping
        layouts replaces the device, like a Switch re-registering it; with
        no halves left the last device is kept for when they come back."""
        layout = self._layout()
        if self.gamepad is not None and layout == self._device_layout:
            return
        if self.gamepad is not None:
            self.gamepad.rumble_cb = None
            self.gamepad.close()
            self.gamepad = None
        if self.motion is not None:
            self.motion.close()
            self.motion = None
        # No player number in the name: Steam keys per-device bindings (e.g.
        # a Capture bind) on it, so it must not change with connect order.
        if layout == "pair":
            name = "Joy-Con 2 Pair"
            mac = (self._halves.get(RIGHT) or next(iter(self._halves.values()))).ctrl.mac
        else:
            name = f"Joy-Con 2 ({'L' if layout == LEFT else 'R'})"
            mac = self._halves[layout].ctrl.mac
        # phys (which SDL uses to tie the pad to its motion device) comes
        # from the half supplying motion.
        self.gamepad = SwitchGamepad(
            name=name,
            button_map=self.button_map,
            product=_LAYOUT_PRODUCTS[layout],
            mac=mac,
            version=_LAYOUT_VERSIONS[layout],
        )
        self.motion = MotionEvdev(name, mac, product=_LAYOUT_PRODUCTS[layout])
        self.gamepad.rumble_cb = self._on_rumble
        self._device_layout = layout
        logger.info("virtual gamepad ready: %s", name)

    def _sync_left_dsu(self) -> None:
        """Hold a DSU slot for the Left half's motion exactly while both halves
        are connected (alone, the Left is already the pair's motion source)."""
        want = (self.dsu is not None and self._extra_dsu_slots is not None
                and LEFT in self._halves and RIGHT in self._halves)
        if want and self._left_dsu_slot is None:
            claim, _release = self._extra_dsu_slots
            self._left_dsu_slot = claim()
            if self._left_dsu_slot is None:
                logger.warning("no free DSU slot for the Left Joy-Con's motion")
                return
            left = self._halves[LEFT].ctrl
            self.dsu.set_slot(self._left_dsu_slot, True, mac=left.mac, battery_mv=left.battery_mv or 0)
            logger.info("Left Joy-Con motion on DSU slot %d (pad %d in emulators)",
                        self._left_dsu_slot, self._left_dsu_slot + 1)
        elif not want and self._left_dsu_slot is not None:
            _claim, release = self._extra_dsu_slots
            self.dsu.set_slot(self._left_dsu_slot, False)
            release(self._left_dsu_slot)
            self._left_dsu_slot = None

    def _update_left_dsu(self, report: P.InputReport, half: _Half) -> None:
        from .bridge import _stick_to_dsu

        lx, ly = half.stick
        lt = 255 if half.buttons & _B["ZL"] else 0
        own = P.InputReport(**{**report.__dict__, "buttons": half.buttons})
        self.dsu.update(self._left_dsu_slot, own,
                        (_stick_to_dsu(lx), _stick_to_dsu(ly), 128, 128), (lt, 0))

    def _update_mouse(self, half: _Half, raw: bytes, now: float) -> None:
        if half.surface.update(on_surface(raw), now):
            logger.info("Joy-Con %s %s", half.side,
                        "on a surface: mouse" if half.surface.active else "lifted: controller")
            mouse = self._mouse_for(half.side)
            if half.surface.active:
                mouse.reset_motion()
            else:
                mouse.release()
        if not half.surface.active:
            return
        left, right, middle = _MOUSE_CLICKS[half.side]
        self._mouse_for(half.side).update(
            raw,
            bool(half.buttons & left),
            bool(half.buttons & right),
            bool(half.buttons & middle),
            half.stick[1],
            now,
        )

    def _mouse_for(self, side: str) -> OpticalMouse:
        mouse = self._mice.get(side)
        if mouse is None:
            mouse = OpticalMouse(f"Joy-Con 2 Mouse ({'L' if side == LEFT else 'R'})")
            self._mice[side] = mouse
        return mouse

    def _pad_contribution(self, half: _Half) -> tuple[int, tuple[float, float]]:
        """This half's buttons + stick for the pad, minus what the mouse uses."""
        buttons, stick = half.buttons, half.stick
        if half.surface.active:
            for mask in _MOUSE_CLICKS[half.side]:
                buttons &= ~mask
            stick = (0.0, 0.0)
        return buttons, stick

    def _motion_side(self) -> str:
        return RIGHT if RIGHT in self._halves else LEFT

    def _merged(self) -> tuple[int, tuple[float, float], tuple[float, float]]:
        if len(self._halves) == 1 and self.sideways:
            (half,) = self._halves.values()
            buttons, stick = self._pad_contribution(half)
            return _sideways(half.side, buttons), _rotate_sideways(half.side, stick), (0.0, 0.0)
        buttons = 0
        lstick = rstick = (0.0, 0.0)
        for half in self._halves.values():
            b, stick = self._pad_contribution(half)
            buttons |= b
            if half.side == LEFT:
                lstick = stick
            else:
                rstick = stick
        return buttons, lstick, rstick

    def _emit(self) -> None:
        if self.gamepad is None:
            return
        buttons, lstick, rstick = self._merged()
        lt = 255 if buttons & _B["ZL"] else 0
        rt = 255 if buttons & _B["ZR"] else 0
        self.gamepad.update(buttons, lstick, rstick, lt, rt)

    def _update_dsu(self, report: P.InputReport) -> None:
        from .bridge import _stick_to_dsu

        buttons, (lx, ly), (rx, ry) = self._merged()
        lt = 255 if buttons & _B["ZL"] else 0
        rt = 255 if buttons & _B["ZR"] else 0
        # DSU reads buttons from the report, so hand it the merged bitmask.
        merged = P.InputReport(**{**report.__dict__, "buttons": buttons})
        sticks = (_stick_to_dsu(lx), _stick_to_dsu(ly), _stick_to_dsu(rx), _stick_to_dsu(ry))
        self.dsu.update(self.slot, merged, sticks, (lt, rt))

    def _on_rumble(self, strong: float, weak: float) -> None:
        with self._lock:
            ctrls = [h.ctrl for h in self._halves.values()]
        for ctrl in ctrls:
            if not ctrl.is_connected:
                continue
            try:
                ctrl.set_rumble(strong, weak)
            except Exception as exc:  # noqa: BLE001
                logger.debug("rumble failed for %s: %s", ctrl.mac, exc)
