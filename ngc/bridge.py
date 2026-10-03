"""Bridge: connect to one or more Switch 2 controllers over raw L2CAP and feed a
uinput virtual gamepad each, with automatic reconnection. Pure userspace; no
BlueZ GATT, no kernel modules.

Connection uses a central BLE scanner: when a saved pad advertises (button press
or Sync), we stop scanning and dial L2CAP immediately. Raw connect cannot run
while the adapter is discovering — including Steam's background scan — so scan
bursts are kept short and always stopped before connect.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json
import logging
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

from bleak import BleakScanner
from bleak.exc import BleakDBusError

from . import att
from . import protocol as P
from .config import CONFIG_DIR, Config, ControllerEntry
from .device import SwitchController
from .dsu import DSUServer
from .gamepad import JOYCON_PAIR_BUTTON_MAP, SwitchGamepad
from .imu_frame import to_dsu, upright
from .joycon import JoyConPair
from .motion_evdev import MotionEvdev
from .status import BridgeState, ControllerState, clear_state, write_state

# Written by system/bazzite-set-player-leds.py when emulator player order changes.
_LED_PLAYERS_PATH = CONFIG_DIR / "led-players.json"
# "split" / "join", written by `python -m ngc joycons ...` (e.g. from the TV
# launcher) and consumed by the running bridge within one status interval.
JOYCON_REQUEST_PATH = CONFIG_DIR / "joycon-request"
# "on" / "off", written by `python -m ngc headset ...`; same pickup.
HEADSET_REQUEST_PATH = CONFIG_DIR / "headset-request"


def _stick_to_dsu(value: float) -> int:
    """Map a calibrated -1.0..1.0 axis to DSU's 0..255 range (128 neutral)."""
    return max(0, min(255, int(round(128 + value * 127))))

logger = logging.getLogger(__name__)

# Most adapters allow only ONE outstanding LE create-connection at a time.
_CONNECT_LOCK = threading.Lock()
_STATUS_INTERVAL_S = 1.5
_SCAN_SETTLE_S = 0.10
# Per-attempt L2CAP connect wait. Short windows fail when Steam keeps LE scan
# busy; after btmgmt stop-find -l a few hundred ms is enough.
_CONNECT_ATTEMPT_S = 0.80
# ~10s of dialing per sighting. Was 16 (~30s), which kept the hub blocked
# on a controller that had already gone back to sleep; a fresh advert
# triggers a new round anyway.
_CONNECT_ATTEMPTS = 6
# Pairing-mode adverts are brief; wake adverts repeat often. A short TTL caused
# missed connects when Sync was held or the 0.25s scan window slipped.
_SEEN_TTL_WAKE_S = 4.0
_SEEN_TTL_PAIRING_S = 45.0
# Recreate BleakScanner if we stay disconnected despite recent adverts.
_HUB_IDLE_RESTART_CYCLES = 80  # ~20s at default scan cadence
# A disconnected controller's virtual pad is kept this long (so a brief link
# drop doesn't make a game lose it), then removed - like a real Bluetooth pad
# disappearing when it sleeps. Kept forever, a sleeping pad's device stayed
# listed ahead of the one in use and apps that pick "the latest" controller
# (the TV launcher's glyphs and evdev reader) latched onto the stale one.
PAD_REMOVE_GRACE_S = 15.0
# Pause between scan bursts while one Joy-Con half is linked and its partner
# is not; keeps the radio mostly free for the live link.
_JOYCON_PARTNER_SCAN_GAP_S = 2.0


def _seen_ttl(mode: str) -> float:
    return _SEEN_TTL_PAIRING_S if mode == "pairing" else _SEEN_TTL_WAKE_S


def _adapter_index() -> str:
    """Prefer hci0; allow override via NGC_HCI (e.g. '1')."""
    return os.environ.get("NGC_HCI", "0").strip() or "0"


_BTMGMT_LOCK = threading.Lock()
_LAST_LE_SCAN_OFF = 0.0
_LE_SCAN_OFF_MIN_INTERVAL_S = 0.35


def _run_quiet(cmd: list[str], *, timeout: float = 2.0) -> None:
    """Best-effort subprocess; never raises into the bridge."""
    try:
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    except Exception:  # noqa: BLE001
        pass


def _force_le_scan_off(*, force: bool = False) -> None:
    """HCI-level LE discovery stop.

    BlueZ ``StopDiscovery`` only ends *our* session. Steam/steamos-manager keeps
    its own session forever (``Discovering`` stays true), which blocks raw L2CAP
    create-connection. ``btmgmt stop-find -l`` stops the controller's LE scan
    regardless of who started it. Requires passwordless ``sudo`` for btmgmt
    (Bazzite default for this user).

    Never raises — a hung btmgmt must not crash the bridge.
    """
    global _LAST_LE_SCAN_OFF
    now = time.monotonic()
    with _BTMGMT_LOCK:
        if not force and (now - _LAST_LE_SCAN_OFF) < _LE_SCAN_OFF_MIN_INTERVAL_S:
            return
        _LAST_LE_SCAN_OFF = now
        idx = _adapter_index()
        # Start detached-ish: kill hung btmgmt so we never block the hub.
        try:
            proc = subprocess.Popen(
                ["sudo", "-n", "btmgmt", "-i", idx, "stop-find", "-l"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=0.5)
                except Exception:  # noqa: BLE001
                    pass
            proc = subprocess.Popen(
                ["sudo", "-n", "btmgmt", "-i", idx, "stop-find"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                proc.wait(timeout=1.5)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=0.5)
                except Exception:  # noqa: BLE001
                    pass
        except Exception:  # noqa: BLE001
            pass


def _bluez_remove_device(mac: str) -> None:
    """Drop BlueZ's Device object so it can't race our raw ATT connect.

    Do not call this while *we* already own a live session for ``mac`` — BlueZ
    treating the ACL as its own Connected device means RemoveDevice tears the
    link down.
    """
    if not mac:
        return
    _run_quiet(["bluetoothctl", "remove", mac], timeout=2.0)
    path = f"/org/bluez/hci{_adapter_index()}/dev_{mac.upper().replace(':', '_')}"
    _run_quiet(
        ["busctl", "call", "org.bluez", f"/org/bluez/hci{_adapter_index()}",
         "org.bluez.Adapter1", "RemoveDevice", "o", path],
        timeout=1.5,
    )


def prepare_bluez_global() -> None:
    """Stop background scanning so raw LE connections can be initiated."""
    subprocess.run(["pkill", "-f", "decky-bluetooth-wake-control"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    _run_quiet(["bluetoothctl", "scan", "off"], timeout=1.5)
    _run_quiet(
        ["busctl", "call", "org.bluez", f"/org/bluez/hci{_adapter_index()}",
         "org.bluez.Adapter1", "StopDiscovery"],
        timeout=1.5,
    )
    _force_le_scan_off(force=True)


def prepare_bluez(mac: str = "", *, remove: bool = False) -> None:
    prepare_bluez_global()
    if remove and mac:
        _bluez_remove_device(mac)


_REORDER_SCRIPTS = [
    "~/.local/bin/bazzite-reorder-on-nso-connect.sh",
    "~/.local/bin/bazzite-dolphin-apply-gcpad1.sh",
    "~/.local/bin/bazzite-eden-reset-controllers.py",
]


def _reorder_enabled() -> bool:
    return os.environ.get("NGC_AUTO_REORDER", "1").lower() not in {"0", "false", "no"}


def _read_led_players() -> dict[str, int]:
    """MAC -> player (1-based) overrides from the LED sync tool."""
    try:
        raw = json.loads(_LED_PLAYERS_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(raw, dict):
        return {}
    out: dict[str, int] = {}
    for mac, player in raw.items():
        try:
            out[str(mac).upper()] = int(player)
        except (TypeError, ValueError):
            continue
    return out


def _led_override_for(mac: str) -> Optional[int]:
    player = _read_led_players().get(mac.upper())
    if player is None:
        return None
    return min(max(player, 1), 8)


def run_emulator_reorder() -> None:
    if not _reorder_enabled():
        return
    for raw in _REORDER_SCRIPTS:
        path = Path(os.path.expanduser(raw))
        if not path.is_file():
            continue
        try:
            cmd = [str(path)]
            if path.suffix == ".py":
                cmd = ["python3", str(path)]
            subprocess.run(cmd, timeout=30,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            logger.info("emulator reorder applied (%s)", path.name)
        except Exception as exc:  # noqa: BLE001
            logger.debug("reorder hook %s failed: %s", path.name, exc)


class _ConnectHub:
    """Single BLE scanner that connects controllers the moment they advertise."""

    def __init__(self, config: Config, stop: threading.Event, bridge: Optional["Bridge"] = None):
        self.config = config
        self.stop = stop
        self.bridge = bridge
        self.host_mac = P.mac_to_int(config.adapter_mac) if config.adapter_mac else None
        self.workers_by_mac: dict[str, "_Worker"] = {}
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._scanner: Optional[BleakScanner] = None
        self._connect_lock: Optional[asyncio.Lock] = None
        self._last_seen: dict[str, tuple[float, str]] = {}
        self._logged: set[str] = set()
        self._logged_foreign: set[str] = set()
        self._executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        self._hub_error = ""
        self._scanning = False
        self._idle_scan_cycles = 0

    def register(self, worker: "_Worker") -> None:
        self.workers_by_mac[worker.entry.mac.upper()] = worker

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run_async, name="ngc-hub", daemon=True)
        self._thread.start()

    def _accept(self, addr: str, adv) -> bool:
        worker = self.workers_by_mac.get(addr)
        if worker is None or worker.is_connected():
            return False
        reconnect = P.reconnect_mac_from_advertisement(adv)
        if reconnect is not None and self.host_mac is not None and reconnect not in (0, self.host_mac):
            # Waking for another host (e.g. a console it was paired to since):
            # only Sync reaches us until it's re-bonded (see _Worker.activate).
            if addr not in self._logged_foreign:
                self._logged_foreign.add(addr)
                logger.info("%s is waking for another host (%012X); hold Sync to bring it back",
                            addr, reconnect)
            return False
        return True

    def _run_async(self) -> None:
        # Keep ONE event loop for the hub's lifetime: bleak's BlueZDBusScannerManager
        # is a process-wide singleton whose D-Bus connection binds to the loop it was
        # first used on, so asyncio.run() per attempt left the manager pinned to a dead loop.
        backoff = 1.0
        fails = 0
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            while not self.stop.is_set():
                try:
                    self._hub_error = ""
                    loop.run_until_complete(self._scan_loop())
                    backoff, fails = 1.0, 0
                except Exception as exc:  # noqa: BLE001
                    self._hub_error = str(exc)
                    fails += 1
                    if fails == 1:
                        logger.exception("connect hub crashed; restarting in %ss", backoff)
                    else:
                        logger.warning(
                            "connect hub restart %d in %ss (%s)", fails, backoff, exc
                        )
                    if isinstance(exc, BleakDBusError) and "InProgress" in str(exc):
                        prepare_bluez_global()
                        _force_le_scan_off()
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 30.0)
        finally:
            loop.close()
            asyncio.set_event_loop(None)

    async def _scan_loop(self) -> None:
        hub = self
        hub._loop = asyncio.get_running_loop()
        hub._connect_lock = asyncio.Lock()
        if hub._executor is None or getattr(hub._executor, "_shutdown", False):
            hub._executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=2, thread_name_prefix="ngc-connect"
            )
        # legacy; use _seen_ttl(mode) below

        def on_adv(device, adv) -> None:
            addr = device.address.upper()
            if hub.stop.is_set() or not hub._accept(addr, adv):
                return
            reconnect = P.reconnect_mac_from_advertisement(adv)
            mode = "pairing" if reconnect == 0 else "wake"
            hub._last_seen[addr] = (time.monotonic(), mode)
            if addr not in hub._logged:
                hub._logged.add(addr)
                logger.info("saw %s (%s)", addr, mode)

        hub._scanner = BleakScanner(detection_callback=on_adv)
        logger.info("scanning for configured controllers (press a button or hold Sync)")
        try:
            while not hub.stop.is_set():
                workers = list(hub.workers_by_mac.values())
                disconnected = [w for w in workers if not w.is_connected()]
                if not disconnected:
                    hub._scanning = False
                    await asyncio.sleep(1.5)
                    continue

                connected_count = len(workers) - len(disconnected)
                joycon_half_missing = any(w.is_joycon() for w in disconnected) and any(
                    w.is_joycon() for w in workers if w.is_connected()
                )
                if connected_count and not joycon_half_missing:
                    # Never scan while a pad holds a live BLE session — scanning for
                    # the other saved pad drops the connected one within seconds.
                    # Exception below: a Joy-Con pair needs both halves linked at
                    # once, so while one half is up and the other is missing we
                    # scan in short, widely spaced bursts instead.
                    hub._scanning = False
                    hub._idle_scan_cycles = 0
                    await asyncio.sleep(2.0)
                    continue

                scan_on_s = 0.50
                hub._scanning = True
                for scan_attempt in range(1, 4):
                    try:
                        await hub._scanner.start()
                        break
                    except BleakDBusError as exc:
                        if "InProgress" not in str(exc):
                            raise
                        hub._scanning = False
                        prepare_bluez_global()
                        _force_le_scan_off()
                        logger.warning(
                            "LE scan held by another client; cleared, retry %d/3",
                            scan_attempt,
                        )
                        await asyncio.sleep(0.5 * scan_attempt)
                        hub._scanning = True
                else:
                    raise BleakDBusError(
                        "org.bluez.Error.InProgress",
                        "LE scan still held by another client after 3 clears",
                    )
                try:
                    await asyncio.sleep(scan_on_s)
                finally:
                    await hub._scanner.stop()
                    hub._scanning = False

                await asyncio.sleep(_SCAN_SETTLE_S)

                now = time.monotonic()
                pending = sorted(
                    [
                        mac for mac, worker in hub.workers_by_mac.items()
                        if not worker.is_connected()
                        and (seen := hub._last_seen.get(mac)) is not None
                        and now - seen[0] <= _seen_ttl(seen[1])
                    ],
                    key=lambda mac: hub._last_seen[mac][0],
                    reverse=True,
                )
                if pending:
                    # Clear Steam's LE scan and drop BlueZ Device ghosts before dialing.
                    prepare_bluez_global()
                    for mac in pending:
                        worker = hub.workers_by_mac.get(mac)
                        if worker is not None and not worker.is_connected():
                            _bluez_remove_device(mac)
                    await asyncio.sleep(0.08)
                async with hub._connect_lock:
                    for mac in pending:
                        worker = hub.workers_by_mac.get(mac)
                        if worker is None or worker.is_connected():
                            hub._logged.discard(mac)
                            hub._last_seen.pop(mac, None)
                            continue
                        mode = hub._last_seen[mac][1]
                        try:
                            ok, detail = await hub._loop.run_in_executor(
                                hub._executor, hub._connect_sync, mac, mode == "pairing"
                            )
                        except Exception as exc:  # noqa: BLE001
                            ok, detail = False, str(exc)
                        if ok:
                            logger.info("connected %s after %s advert", mac, mode)
                            hub._last_seen.pop(mac, None)
                            hub._logged.discard(mac)
                        else:
                            logger.info("connect to %s (%s) failed (%s)", mac, mode, detail)
                            # Forget this sighting so we only re-dial on a fresh
                            # advert, not a stale one (pairing ones live 45s).
                            hub._last_seen.pop(mac, None)
                            hub._logged.discard(mac)
                            _force_le_scan_off(force=True)

                for mac, (seen_at, mode) in list(hub._last_seen.items()):
                    if now - seen_at > _seen_ttl(mode):
                        hub._last_seen.pop(mac, None)
                        hub._logged.discard(mac)

                if disconnected and not pending:
                    hub._idle_scan_cycles += 1
                    if hub._idle_scan_cycles >= _HUB_IDLE_RESTART_CYCLES:
                        logger.warning(
                            "hub idle with %d disconnected pad(s); restarting BLE scanner",
                            len(disconnected),
                        )
                        hub._idle_scan_cycles = 0
                        hub._last_seen.clear()
                        hub._logged.clear()
                        prepare_bluez_global()
                        try:
                            await hub._scanner.stop()
                        except Exception:
                            pass
                        hub._scanner = BleakScanner(detection_callback=on_adv)
                else:
                    hub._idle_scan_cycles = 0

                if joycon_half_missing:
                    await asyncio.sleep(_JOYCON_PARTNER_SCAN_GAP_S)
                else:
                    await asyncio.sleep(0.05 if connected_count else 0.025)
        finally:
            hub._scanning = False
            if hub._scanner is not None:
                await hub._scanner.stop()

    def _connect_sync(self, mac: str, pairing: bool = False) -> tuple[bool, str]:
        worker = self.workers_by_mac.get(mac)
        if worker is None or worker.is_connected():
            return False, "already connected"
        adapter = self.config.adapter_mac
        if not adapter:
            return False, "no adapter configured"
        attempt_s = _CONNECT_ATTEMPT_S
        dst_types = (
            (worker.last_dst_type, att.LE_RANDOM if worker.last_dst_type == att.LE_PUBLIC else att.LE_PUBLIC)
            if worker.last_dst_type is not None
            else (att.LE_PUBLIC, att.LE_RANDOM)
        )
        with _CONNECT_LOCK:
            last_detail = "no attempts"
            for attempt in range(_CONNECT_ATTEMPTS):
                ctrl = SwitchController(mac, adapter)
                ctrl.want_extended = self.config.pro2_headset_mic
                ctrl.GC_IMPACT_THRESHOLD = self.config.gc_impact_threshold
                for dst in dst_types:
                    ok, detail = self._connect_dst_with_polling(ctrl, dst, attempt_s)
                    if ok:
                        ctrl.att.dst_type = dst
                        worker.last_dst_type = dst
                        if worker.activate(ctrl, rebond=pairing):
                            worker._ready.set()
                            return True, "ok"
                        ctrl.close()
                        return False, "session setup failed"
                    last_detail = detail
                ctrl.close()
                time.sleep(0.02)
            if worker.last_dst_type is not None:
                worker.last_dst_type = None
            return False, last_detail

    @staticmethod
    def _connect_dst_with_polling(ctrl: SwitchController, dst: int, attempt_s: float) -> tuple[bool, str]:
        _force_le_scan_off()
        return ctrl.att._connect_once(dst, attempt_s)


class _Worker:
    """Owns input streaming, rumble, and virtual gamepad for one controller session."""

    def __init__(
        self,
        entry: ControllerEntry,
        config: Config,
        stop: threading.Event,
        hub: _ConnectHub,
        dsu: Optional[DSUServer] = None,
        on_topology_change: Optional[callable] = None,
    ):
        self.entry = entry
        self.config = config
        self._stop = stop
        self.hub = hub
        self.dsu = dsu
        self.on_topology_change = on_topology_change
        self.slot = max(0, min(3, entry.player - 1))
        self.gamepad: Optional[SwitchGamepad] = None
        self.motion: Optional[MotionEvdev] = None
        self._gamepad_product: Optional[int] = None
        self.controller: Optional[SwitchController] = None
        self._disconnected = threading.Event()
        self._ready = threading.Event()
        self._led_player: Optional[int] = None
        self.last_dst_type: Optional[int] = None
        # Set while this Joy-Con feeds the bridge's combined L+R pad instead
        # of owning a gamepad (see joycon.JoyConPair).
        self.pair: Optional[JoyConPair] = None
        # Product ID from the most recent session (kept across disconnects).
        self.product_id: Optional[int] = None
        # Player number held for the current session (see Bridge.claim_player).
        self.player: Optional[int] = None
        # When the last session ended, while its pad is still kept around.
        self._disconnected_at: Optional[float] = None
        # When the current session came up; orders split Joy-Cons' players.
        self.connected_at: float = 0.0
        # The Joy-Con pad this worker's player number was claimed for.
        self._pair_owner: Optional[JoyConPair] = None
        self._pad_lock = threading.Lock()

    def is_connected(self) -> bool:
        return self.controller is not None and self.controller.is_connected

    def is_joycon(self) -> bool:
        """True for a Joy-Con 2 entry, known from its last session or, before
        it has ever connected here, from the name saved at pairing."""
        if self.product_id is not None:
            return P.is_joycon(self.product_id)
        return "joy-con" in (self.entry.name or "").lower()

    def joycon_side(self) -> Optional[str]:
        """"left" / "right" for a Joy-Con 2 (from its last session, else the
        name saved at pairing), None for anything else."""
        from .joycon import LEFT, RIGHT, side_of
        if self.product_id is not None:
            return side_of(self.product_id)
        name = (self.entry.name or "").lower()
        if "joy-con" not in name:
            return None
        return LEFT if "left" in name else RIGHT if "right" in name else None

    def _claim_player(self) -> int:
        """Switch-style numbering: the lowest player number no other connected
        controller holds (a Joy-Con pair counts as one). A led-players.json
        override still wins when present."""
        override = _led_override_for(self.entry.mac)
        if override is not None:
            return override
        bridge = self.hub.bridge
        if bridge is None:
            return self.entry.player
        if self.is_joycon():
            self._pair_owner = bridge.joycon_pair_for(self.joycon_side())
            return bridge.claim_player(self._pair_owner)
        return bridge.claim_player(self)

    def _release_player(self) -> None:
        self.player = None
        bridge = self.hub.bridge
        if bridge is None:
            return
        bridge.release_player(self)
        pair, self._pair_owner = self._pair_owner, None
        if pair is not None and pair.is_empty():
            bridge.release_player(pair)

    def effective_player(self) -> int:
        """Config player slot, optionally overridden by led-players.json."""
        override = _led_override_for(self.entry.mac)
        return override if override is not None else self.entry.player

    def _on_input(self, ctrl: SwitchController, report: P.InputReport) -> None:
        if self.pair is not None:
            self.pair.on_input(ctrl, report)
            return
        (lx, ly), (rx, ry), lt, rt = ctrl.calibrated_input(report)
        if self.gamepad is not None:
            self.gamepad.update(report.buttons, (lx, ly), (rx, ry), lt, rt)
        # The Pro 2 shares the Joy-Con 2's raw IMU frame; give emulators the
        # same SDL / DSU frames the Joy-Cons get (see imu_frame). The
        # GameCube pad's motion is left as it was.
        if ctrl.product_id == P.PRO_CONTROLLER2_PID:
            imu = upright(report)
            dsu_imu = to_dsu(imu)
        else:
            imu = dsu_imu = report
        if self.motion is not None:
            self.motion.update(imu)
        if self.dsu is not None:
            sticks = (_stick_to_dsu(lx), _stick_to_dsu(ly),
                      _stick_to_dsu(rx), _stick_to_dsu(ry))
            self.dsu.update(self.slot, dsu_imu, sticks, (lt, rt))

    def _on_disconnect(self) -> None:
        logger.warning("controller %s disconnected", self.entry.mac)
        self._disconnected.set()

    def _on_rumble(self, strong: float, weak: float) -> None:
        ctrl = self.controller
        if ctrl is None or not ctrl.is_connected:
            return
        try:
            ctrl.set_rumble(strong, weak)
        except Exception as exc:  # noqa: BLE001
            logger.debug("rumble failed: %s", exc)

    def remove_stale_pad(self, now: float) -> None:
        """Drop the kept pad once the controller has been gone past the grace
        period (see PAD_REMOVE_GRACE_S); the next session recreates it."""
        with self._pad_lock:
            if (self._disconnected_at is None or self.is_connected()
                    or now - self._disconnected_at < PAD_REMOVE_GRACE_S):
                return
            self._disconnected_at = None
            if self.gamepad is not None:
                self.gamepad.rumble_cb = None
                self.gamepad.close()
                self.gamepad = None
                self._gamepad_product = None
                logger.info("removed virtual pad for %s (gone %.0fs)", self.entry.mac, PAD_REMOVE_GRACE_S)
            if self.motion is not None:
                self.motion.close()
                self.motion = None

    def _ensure_gamepad(self, ctrl: SwitchController) -> None:
        with self._pad_lock:
            self._disconnected_at = None
            self._ensure_gamepad_locked(ctrl)

    def _ensure_gamepad_locked(self, ctrl: SwitchController) -> None:
        pid = ctrl.product_id
        bridge = self.hub.bridge
        if P.is_joycon(pid) and bridge is not None:
            # Normally claimed for this pad already; covers a Joy-Con whose
            # saved name didn't identify it as one before this session.
            bridge.release_player(self)
            self.move_to_pair(bridge.joycon_pair_for(self.joycon_side()))
            return
        if self.gamepad is not None and self._gamepad_product == pid:
            return
        if self.gamepad is not None:
            self.gamepad.rumble_cb = None
            self.gamepad.close()
            self.gamepad = None
            self._gamepad_product = None
        if self.motion is not None:
            self.motion.close()
            self.motion = None
        # No player number in the name: numbers follow connection order, and
        # Steam keys per-device bindings on the name.
        name = ctrl.name
        self.gamepad = SwitchGamepad(
            name=name,
            button_map=self.config_button_map(pid),
            product=pid,
            mac=self.entry.mac,
        )
        self.motion = MotionEvdev(name, self.entry.mac, product=pid)
        self._gamepad_product = pid
        logger.info("virtual gamepad ready: %s", name)

    def config_button_map(self, product_id: int):
        from .gamepad import button_map_for_product
        from evdev import ecodes as e

        if not self.config.button_map:
            return button_map_for_product(product_id)
        resolved = {}
        for switch_name, code in self.config.button_map.items():
            resolved[switch_name] = getattr(e, code) if isinstance(code, str) else code
        return resolved

    def activate(self, ctrl: SwitchController, rebond: bool = False) -> bool:
        """Run the handshake and bring the session up. ``rebond`` is set when
        the controller reached us in pairing mode (Sync held): its stored host
        may no longer be us, so write the bond again or button-press wake
        keeps going to whatever host it holds instead."""
        mac = self.entry.mac
        try:
            logger.info("connected to %s (MTU %d)", mac, ctrl.att.mtu)
            ctrl.input_callback = self._on_input
            ctrl.disconnect_callback = self._on_disconnect
            self._disconnected.clear()
            player = self._claim_player()
            self.player = player
            self.slot = max(0, min(3, player - 1))
            ctrl.initialize(player=player)
            self._led_player = player
            self.product_id = ctrl.product_id
            if ctrl.extended and self.hub.bridge is not None:
                mic = self.hub.bridge.headset_mic_for(self.entry.mac)
                if mic is not None:
                    ctrl.mic_callback = mic.feed
            self.connected_at = time.monotonic()
            if rebond or not self.entry.bonded:
                ctrl.bond()
                self.config.mark_bonded(mac, True)
                self.config.save()
                self.entry.bonded = True
                logger.info("bonded %s to %s", mac, self.config.adapter_mac)
            self.controller = ctrl
            self._ensure_gamepad(ctrl)
            if self.gamepad is not None and self.config.enable_rumble:
                self.gamepad.rumble_cb = self._on_rumble
            if self.dsu is not None and self.pair is None:
                self.dsu.set_slot(self.slot, True, mac=mac, battery_mv=ctrl.battery_mv or 0)
            if self.on_topology_change is not None:
                self.on_topology_change()
            if self.hub.bridge is not None:
                self.hub.bridge._publish_state()
            return True
        except Exception as exc:  # noqa: BLE001
            logger.error("session setup failed for %s: %s", mac, exc)
            self._teardown_partial(ctrl)
            return False

    def move_to_pair(self, pair: JoyConPair, relayout: bool = True) -> None:
        """Feed this Joy-Con into ``pair`` under that pad's player number."""
        ctrl = self.controller
        if ctrl is None:
            return
        bridge = self.hub.bridge
        player = bridge.claim_player(pair) if bridge is not None else pair.player
        self._pair_owner = pair
        self.player = player
        if self._led_player != player:
            ctrl.set_player_leds(player)
            self._led_player = player
        pair.attach(ctrl, relayout=relayout)
        self.pair = pair

    def _detach_pair(self, ctrl: Optional[SwitchController]) -> None:
        if self.pair is not None and ctrl is not None:
            self.pair.detach(ctrl)
        self.pair = None

    def _teardown_partial(self, ctrl: Optional[SwitchController] = None) -> None:
        self._detach_pair(ctrl)
        self._release_player()
        if ctrl is not None:
            try:
                ctrl.close()
            except Exception:  # noqa: BLE001
                pass
        if self.gamepad is not None:
            self.gamepad.rumble_cb = None
            self.gamepad.close()
            self.gamepad = None
            self._gamepad_product = None
        if self.motion is not None:
            self.motion.close()
            self.motion = None

    def _teardown_session(self, *, full: bool = False) -> None:
        paired = self.pair is not None
        self._detach_pair(self.controller)
        if self.gamepad is not None:
            self.gamepad.rumble_cb = None
            if full:
                self.gamepad.close()
                self.gamepad = None
                self._gamepad_product = None
            else:
                self.gamepad.release_all()
                self._disconnected_at = time.monotonic()
        if self.motion is not None:
            if full:
                self.motion.close()
                self.motion = None
        if self.dsu is not None and not paired:
            self.dsu.set_slot(self.slot, False)
        if self.controller:
            self.controller.close()
            self.controller = None
        self._release_player()
        if self.is_joycon() and self.hub.bridge is not None:
            self.hub.bridge.joycon_disconnected()
        if self.on_topology_change is not None:
            self.on_topology_change()
        if self.hub.bridge is not None:
            self.hub.bridge._publish_state()

    def _idle_sleep_s(self) -> float:
        return max(0.0, float(getattr(self.config, "idle_sleep_s", 300.0)))

    def _sleep_for_idle(self) -> None:
        ctrl = self.controller
        if ctrl is None:
            self._disconnected.set()
            return
        try:
            ctrl.sleep()
        except Exception as exc:  # noqa: BLE001
            logger.warning("idle sleep failed for %s (%s)", self.entry.mac, exc)
            try:
                ctrl.close()
            except Exception:  # noqa: BLE001
                pass
        self._disconnected.set()

    def run(self) -> None:
        self.hub.register(self)
        while not self._stop.is_set():
            self._ready.clear()
            while not self._stop.is_set() and not self._ready.wait(1.0):
                pass
            if self._stop.is_set() or not self.is_connected():
                continue
            idle_sleep_s = self._idle_sleep_s()
            while not self._stop.is_set() and not self._disconnected.is_set():
                if idle_sleep_s > 0 and self.controller is not None:
                    idle_for = time.monotonic() - self.controller.last_button_at
                    if idle_for >= idle_sleep_s:
                        logger.info(
                            "controller %s idle %.0fs with no button presses; sleeping",
                            self.entry.mac,
                            idle_for,
                        )
                        self._sleep_for_idle()
                        break
                self._disconnected.wait(0.5)
            self._teardown_session(full=False)

    def cleanup(self) -> None:
        self._teardown_session(full=True)


class Bridge:
    def __init__(self, config: Config):
        self.config = config
        self._stop = threading.Event()
        self.workers: list[_Worker] = []
        self.dsu: Optional[DSUServer] = None
        self.hub = _ConnectHub(config, self._stop, bridge=self)
        self._reorder_timer: Optional[threading.Timer] = None
        self._reorder_lock = threading.Lock()
        self._state_lock = threading.Lock()
        # Joy-Con pads: "pair" while combined, "left" / "right" while split.
        self._joycon_pairs: dict[str, JoyConPair] = {}
        self._joycon_pair_lock = threading.Lock()
        self._joycon_switch_lock = threading.Lock()
        self._joycons_split = False
        # Pro Controller 2 headset mics by MAC: (HeadsetMic, VirtualMic).
        self._headset_mics: dict = {}
        self._players: dict[object, int] = {}
        self._players_lock = threading.Lock()
        self._extra_dsu_slots: set[int] = set()

    def joycon_pair_for(self, side: Optional[str]) -> JoyConPair:
        """The pad a Joy-Con feeds: the shared pair while combined, its own
        sideways pad while split (see set_joycon_split). Created on first use."""
        key = side if self._joycons_split and side is not None else "pair"
        with self._joycon_pair_lock:
            pair = self._joycon_pairs.get(key)
            if pair is None:
                button_map = (
                    self.workers[0].config_button_map(P.PRO_CONTROLLER2_PID)
                    if self.config.button_map
                    else JOYCON_PAIR_BUTTON_MAP
                )
                pair = JoyConPair(
                    button_map,
                    dsu=self.dsu,
                    sideways=self.config.joycon_single_sideways,
                    mouse=self.config.joycon_mouse,
                    extra_dsu_slots=(self.claim_extra_dsu_slot, self.release_extra_dsu_slot),
                )
                self._joycon_pairs[key] = pair
            return pair

    def set_joycon_split(self, split: bool) -> None:
        """Split a Joy-Con pair into two independent sideways controllers, or
        join them back, while connected. Each split Joy-Con gets its own
        player number (lights) in the order the two connected; joined, the
        pair takes the lowest free number. Not saved: once no Joy-Con is
        connected they go back to pairing (see joycon_disconnected)."""
        with self._joycon_switch_lock:
            if split == self._joycons_split:
                return
            moving = sorted(
                (w for w in self.workers if w.pair is not None and w.controller is not None),
                key=lambda w: w.connected_at,
            )
            old = {id(w.pair): w.pair for w in moving}
            for w in moving:
                w.pair.detach(w.controller, relayout=False)
            now = time.monotonic()
            for pair in old.values():
                self.release_player(pair)
                pair.remove_stale_pad(now, 0.0)
            self._joycons_split = split
            new = {}
            for w in moving:
                pair = self.joycon_pair_for(w.joycon_side())
                w.move_to_pair(pair, relayout=False)
                new[id(pair)] = pair
            for pair in new.values():
                pair.sync_device()
            logger.info("Joy-Cons %s", "split into separate controllers" if split else "joined into a pair")
        self._publish_state()

    def joycon_disconnected(self) -> None:
        """Once every Joy-Con is off, a split ends: the next ones to connect
        pair up again, like a Switch, until they're split again."""
        with self._joycon_switch_lock:
            if not self._joycons_split:
                return
            if any(w.is_joycon() and w.is_connected() for w in self.workers):
                return
            self._joycons_split = False
            logger.info("all Joy-Cons off; they'll pair again when they reconnect")

    def headset_mic_for(self, mac: str):
        """The decoder feeding this Pro Controller 2's virtual microphone
        (created on first use, kept across reconnects so apps keep their
        input selected); None if libopus or PipeWire/pactl isn't available."""
        from .headset_mic import HeadsetMic, VirtualMic

        with self._joycon_pair_lock:
            if mac in self._headset_mics:
                return self._headset_mics[mac][0]
            try:
                n = len(self._headset_mics) + 1
                name = "Pro Controller 2 Headset Mic" + (f" {n}" if n > 1 else "")
                virtual = VirtualMic(f"switch2_headset_mic_{n}", name)
                mic = HeadsetMic(virtual)
            except Exception as exc:  # noqa: BLE001
                logger.warning("headset mic unavailable: %s", exc)
                return None
            self._headset_mics[mac] = (mic, virtual)
            return mic

    def _check_headset_request(self) -> None:
        try:
            request = HEADSET_REQUEST_PATH.read_text().strip()
            HEADSET_REQUEST_PATH.unlink()
        except OSError:
            return
        if request in ("on", "off"):
            threading.Thread(target=self.set_headset_mode, args=(request == "on",),
                             name="headset-mode", daemon=True).start()

    def set_headset_mode(self, on: bool) -> None:
        """Turn Pro Controller 2 headset mode on/off (saved), switching any
        connected Pro 2 live. If one doesn't start sending the new reports
        within a few seconds, its link is dropped so it reconnects in the
        right mode."""
        self.config.pro2_headset_mic = on
        self.config.save()
        logger.info("headset mode %s for Pro Controller 2", "on" if on else "off")
        for worker in list(self.workers):
            ctrl = worker.controller
            if ctrl is None or not ctrl.is_connected or ctrl.product_id != P.PRO_CONTROLLER2_PID:
                continue
            ctrl.want_extended = on
            try:
                switched = ctrl.set_headset_mode(on)
            except Exception as exc:  # noqa: BLE001
                logger.warning("live headset switch failed for %s: %s", worker.entry.mac, exc)
                switched = False
            if switched:
                mic = self.headset_mic_for(worker.entry.mac) if on else None
                ctrl.mic_callback = mic.feed if mic is not None else None
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and ctrl.last_input_at == 0.0:
                    time.sleep(0.1)
                if ctrl.last_input_at != 0.0:
                    continue
                logger.info("%s didn't switch live; reconnecting it", worker.entry.mac)
            ctrl.close()  # the worker sees the drop; the next connect uses want_extended

    def _check_joycon_request(self) -> None:
        """Pick up a split/join written by `python -m ngc joycons ...`."""
        try:
            request = JOYCON_REQUEST_PATH.read_text().strip()
            JOYCON_REQUEST_PATH.unlink()
        except OSError:
            return
        if request in ("split", "join"):
            self.set_joycon_split(request == "split")

    def claim_player(self, owner) -> int:
        """Give a connecting controller (a worker, or the Joy-Con pair) the
        lowest free player number, like a Switch. Idempotent per owner."""
        with self._players_lock:
            player = self._players.get(owner)
            if player is None:
                used = set(self._players.values())
                player = next((p for p in range(1, 9) if p not in used), 8)
                self._players[owner] = player
                logger.info("assigned P%d", player)
        if isinstance(owner, JoyConPair):
            owner.set_player(player)
        return player

    def release_player(self, owner) -> None:
        with self._players_lock:
            self._players.pop(owner, None)

    def claim_extra_dsu_slot(self) -> Optional[int]:
        """A DSU slot outside every connected player's own (player - 1),
        highest first so it stays clear of players joining later. Used for a
        Joy-Con pair's left-half motion; None when all four are taken."""
        with self._players_lock:
            used = {p - 1 for p in self._players.values()} | self._extra_dsu_slots
            slot = next((s for s in (3, 2, 1, 0) if s not in used), None)
            if slot is not None:
                self._extra_dsu_slots.add(slot)
            return slot

    def release_extra_dsu_slot(self, slot: int) -> None:
        with self._players_lock:
            self._extra_dsu_slots.discard(slot)

    # Measured empty on a real NSO GameCube pad (~2939 mV before cutoff).
    BATTERY_EMPTY_MV = 2950
    BATTERY_FULL_MV = 4200

    def _battery_pct(self, mv: Optional[int]) -> Optional[int]:
        if not mv:
            return None
        span = self.BATTERY_FULL_MV - self.BATTERY_EMPTY_MV
        return max(0, min(100, int((mv - self.BATTERY_EMPTY_MV) * 100 / span)))

    def _publish_state(self) -> None:
        entries = self.config.entries()
        connected = sum(1 for w in self.workers if w.is_connected())
        with self._state_lock:
            controllers: list[ControllerState] = []
            for entry in entries:
                worker = next(
                    (w for w in self.workers if w.entry.mac.upper() == entry.mac.upper()),
                    None,
                )
                ctrl = worker.controller if worker else None
                mv = ctrl.battery_mv if ctrl else None
                controllers.append(
                    ControllerState(
                        mac=entry.mac,
                        player=(worker.player if worker and worker.player else entry.player),
                        name=entry.name or (ctrl.name if ctrl else ""),
                        bonded=entry.bonded,
                        connected=worker.is_connected() if worker else False,
                        battery_pct=self._battery_pct(mv),
                        battery_mv=mv,
                    )
                )
            if self._stop.is_set():
                headline, detail, service = "Stopping", "", "stopping"
            elif self.hub._hub_error:
                headline, detail, service = "Needs attention", self.hub._hub_error[:120], "error"
            elif not entries:
                headline, detail, service = "Set up", "Add a controller once with Sync.", "running"
            elif connected:
                names = ", ".join(
                    f"P{c.player} {c.name or 'Controller'}"
                    for c in controllers if c.connected
                )
                headline = f"{connected} connected"
                detail = f"{names} — ready in Steam and emulators"
                service = "running"
            elif self.hub._scanning:
                headline = "Scanning"
                detail = "Press a button or hold Sync on a saved controller."
                service = "running"
            else:
                headline = "Ready"
                detail = "Press a button or hold Sync on a saved controller."
                service = "running"
            write_state(
                BridgeState(
                    hub_alive=not self._stop.is_set() and not self.hub._hub_error,
                    hub_scanning=self.hub._scanning,
                    hub_error=self.hub._hub_error,
                    service=service,
                    headline=headline,
                    detail=detail,
                    controllers=controllers,
                    joycons_split=self._joycons_split,
                    joycons_connected=sum(
                        1 for w in self.workers if w.is_joycon() and w.is_connected()
                    ),
                )
            )

    def _apply_led_overrides(self) -> None:
        """Push led-players.json slots onto connected bridge pads."""
        mapping = _read_led_players()
        if not mapping:
            return
        for worker in self.workers:
            ctrl = worker.controller
            if ctrl is None or not ctrl.is_connected:
                continue
            player = mapping.get(worker.entry.mac.upper())
            if player is None or worker._led_player == player:
                continue
            try:
                ctrl.set_player_leds(player)
                worker._led_player = player
                logger.info("player LEDs %s -> P%d (led-players.json)", worker.entry.mac, player)
            except Exception as exc:  # noqa: BLE001
                logger.debug("player LED update failed for %s: %s", worker.entry.mac, exc)

    def _state_loop(self) -> None:
        while not self._stop.wait(_STATUS_INTERVAL_S):
            try:
                self._apply_led_overrides()
                now = time.monotonic()
                for worker in self.workers:
                    worker.remove_stale_pad(now)
                for pair in list(self._joycon_pairs.values()):
                    pair.remove_stale_pad(now, PAD_REMOVE_GRACE_S)
                self._check_joycon_request()
                self._check_headset_request()
                self._publish_state()
            except Exception as exc:  # noqa: BLE001
                logger.debug("state publish failed: %s", exc)

    def _schedule_reorder(self) -> None:
        if self._stop.is_set():
            return
        with self._reorder_lock:
            if self._reorder_timer is not None:
                self._reorder_timer.cancel()
            self._reorder_timer = threading.Timer(2.0, run_emulator_reorder)
            self._reorder_timer.daemon = True
            self._reorder_timer.start()

    def run(self) -> None:
        entries = self.config.entries()
        if not entries:
            raise RuntimeError("no controllers configured (run pairing first)")

        prepare_bluez_global()

        self.dsu = DSUServer()
        if not self.dsu.start():
            self.dsu = None

        logger.info("starting %d controller worker(s)", len(entries))
        for entry in entries:
            worker = _Worker(
                entry,
                self.config,
                self._stop,
                self.hub,
                dsu=self.dsu,
                on_topology_change=self._schedule_reorder,
            )
            self.workers.append(worker)
            self.hub.register(worker)
            threading.Thread(target=worker.run, name=f"ctrl-{entry.player}", daemon=True).start()

        self.hub.start()
        self._publish_state()
        threading.Thread(target=self._state_loop, name="ngc-state", daemon=True).start()

        while not self._stop.is_set():
            self._stop.wait(0.5)

        with self._reorder_lock:
            if self._reorder_timer is not None:
                self._reorder_timer.cancel()
        for worker in self.workers:
            worker.cleanup()
        for pair in self._joycon_pairs.values():
            pair.close()
        for mic, virtual in self._headset_mics.values():
            mic.close()
            virtual.close()
        if self.dsu is not None:
            self.dsu.stop()
        if self.hub._executor is not None:
            self.hub._executor.shutdown(wait=False, cancel_futures=True)
        clear_state()

    def stop(self) -> None:
        self._stop.set()

    def pulse_gamecube_hotkey(self, *switch_names: str, hold_s: float = 0.12) -> None:
        """Briefly press mapped buttons on connected GameCube pads (e.g. C+R for Dolphin save)."""
        masks = 0
        for name in switch_names:
            masks |= P.SWITCH_BUTTONS.get(name, 0)
        if not masks:
            return
        for worker in self.workers:
            gp = worker.gamepad
            ctrl = worker.controller
            if gp is None or ctrl is None or not worker.is_connected():
                continue
            if ctrl.product_id != P.NSO_GAMECUBE_PID:
                continue
            gp.update(masks, (0.0, 0.0), (0.0, 0.0), 0, 0)
            time.sleep(hold_s)
            gp.update(0, (0.0, 0.0), (0.0, 0.0), 0, 0)
