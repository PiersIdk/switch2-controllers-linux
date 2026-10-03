#!/usr/bin/env python3
"""Guided raw-report capture for a single Joy-Con 2 over the raw ATT path.

Connects exactly the way the bridge connects a Pro Controller 2 (no BlueZ
GATT, no patched bluetoothd), runs the normal handshake, then walks through
timed prompts ("press A", "move the stick", ...) and logs every input report
as hex with the prompt it was captured under. The log is what the Joy-Con
mapping gets built from.

    .venv312/bin/python tools/jc2_capture.py            # pairing-mode scan
    .venv312/bin/python tools/jc2_capture.py AA:BB:...  # known MAC

Hold Sync on the Joy-Con until its LEDs sweep before running. The nso-gc
user service is paused for the duration (it shares the adapter) and
restarted afterwards. Does not bond, so the Joy-Con's existing pairings are
left alone.
"""

from __future__ import annotations

import asyncio
import logging
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ngc import protocol as P
from ngc.bridge import prepare_bluez
from ngc.config import detect_adapter
from ngc.device import SwitchController
from ngc.scanner import find_first

LOG_DIR = Path(__file__).resolve().parent.parent / "captures"
PHASE_S = 4.0
GAP_S = 1.5

COMMON_PHASES = [
    ("idle", "Leave it flat and still, touch nothing"),
    ("SL", "Hold SL (the small rail button nearer the top)"),
    ("SR", "Hold SR (the small rail button nearer the bottom)"),
    ("stick_circle", "Slowly roll the stick around its full edge, twice"),
    ("stick_press", "Click the stick down and hold"),
    ("mouse_move", "Put it rail-side down on a desk/mousepad and slide it around"),
    ("tilt", "Pick it up and rotate/tilt it in every direction"),
]
RIGHT_PHASES = [
    ("A", "Hold A"), ("B", "Hold B"), ("X", "Hold X"), ("Y", "Hold Y"),
    ("R", "Hold R"), ("ZR", "Hold ZR"), ("PLUS", "Hold +"),
    ("HOME", "Hold Home (tap-hold briefly)"), ("C", "Hold C"),
]
LEFT_PHASES = [
    ("UP", "Hold Up"), ("DOWN", "Hold Down"), ("LEFT", "Hold Left"), ("RIGHT", "Hold Right"),
    ("L", "Hold L"), ("ZL", "Hold ZL"), ("MINUS", "Hold -"), ("CAPTURE", "Hold Capture"),
]


def _service(action: str) -> None:
    subprocess.run(["systemctl", "--user", action, "nso-gc.service"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _say(msg: str) -> None:
    print(msg, flush=True)


async def _discover(mac: str | None) -> tuple[str, int]:
    if mac:
        return mac, 0
    _say("Scanning for a Joy-Con 2 in pairing mode (hold Sync until the LEDs sweep)...")
    found = await find_first(timeout=40.0, require_pairing=True,
                             only_pids={P.JOYCON2_LEFT_PID, P.JOYCON2_RIGHT_PID})
    if not found:
        raise SystemExit("No Joy-Con 2 seen in pairing mode. Hold Sync and run again.")
    _say(f"Found {found.name} at {found.device.address}")
    return found.device.address, found.product_id


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    arg_mac = sys.argv[1] if len(sys.argv) > 1 else None
    adapter = detect_adapter()
    if not adapter:
        raise SystemExit("No Bluetooth adapter found.")

    _service("stop")
    try:
        mac, pid = asyncio.run(_discover(arg_mac))
        return _capture(mac, pid, adapter)
    finally:
        _service("start")


def _capture(mac: str, scan_pid: int, adapter: str) -> int:
    LOG_DIR.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")

    prepare_bluez(mac, remove=True)
    ctrl = SwitchController(mac, adapter)
    _say("Connecting over raw ATT...")
    t0 = time.monotonic()
    for attempt in range(1, 16):
        if ctrl.connect(timeout=6):
            break
        _say(f"  connect attempt {attempt} failed, retrying")
    else:
        _say("Could not establish the raw link. Keep it in pairing mode and retry.")
        return 1
    _say(f"Link up after {time.monotonic() - t0:.1f}s; running Pro 2 handshake...")

    lock = threading.Lock()
    state = {"phase": "handshake"}
    rows: list[str] = []

    def on_input(_c, report: P.InputReport) -> None:
        with lock:
            rows.append(f"{time.monotonic():.3f}\t{state['phase']}\t{len(report.raw)}\t{report.raw.hex(' ')}")

    ctrl.input_callback = on_input
    try:
        ctrl.initialize(player=1)
    except Exception as exc:  # noqa: BLE001
        _say(f"HANDSHAKE FAILED: {exc}")
        ctrl.close()
        return 1

    info = ctrl.info
    pid = info.product_id if info else scan_pid
    side = "left" if pid == P.JOYCON2_LEFT_PID else "right" if pid == P.JOYCON2_RIGHT_PID else "unknown"
    _say(f"Handshake OK: {info.name if info else '?'} serial={info.serial_number if info else '?'} "
         f"pid={pid:#06x} side={side}")
    _say(f"  calib L={ctrl.left_calib}  R={ctrl.right_calib}  vib_handle={getattr(ctrl, 'h_vibration', None)}")

    # Pro 2 handshake enables buttons/sticks/motion only; add the optical
    # mouse bit separately so a failure here can't mask a working handshake.
    try:
        ctrl.enable_features(0x03 | P.FEATURE_MOTION | P.FEATURE_MOUSE)
        mouse_note = "mouse feature enabled"
    except Exception as exc:  # noqa: BLE001
        mouse_note = f"mouse feature enable FAILED: {exc}"
    _say(mouse_note)

    phases = (LEFT_PHASES if side == "left" else RIGHT_PHASES) + COMMON_PHASES
    _say(f"\nStarting {len(phases)} prompts, {PHASE_S:.0f}s each. Hold the Joy-Con upright (stick at top).\n")
    time.sleep(2.0)
    for i, (label, prompt) in enumerate(phases, 1):
        with lock:
            state["phase"] = "release"
        _say(f"[{i}/{len(phases)}] next: {prompt}")
        time.sleep(GAP_S)
        with lock:
            state["phase"] = label
        _say("    >>> NOW")
        time.sleep(PHASE_S)
        if not ctrl.is_connected:
            _say("!! Joy-Con disconnected mid-capture")
            break
    with lock:
        state["phase"] = "done"
    _say("\nDone, let go. Closing link.")
    ctrl.close()

    out = LOG_DIR / f"jc2-{side}-{stamp}.tsv"
    header = [
        f"# mac={mac} pid={pid:#06x} side={side} adapter={adapter}",
        f"# info={info}",
        f"# calib_L={ctrl.left_calib} calib_R={ctrl.right_calib}",
        f"# {mouse_note}",
        "# t\tphase\tlen\thex",
    ]
    out.write_text("\n".join(header + rows) + "\n", encoding="utf-8")
    _say(f"Saved {len(rows)} reports to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
