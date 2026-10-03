#!/usr/bin/env python3
"""Look for the Pro Controller 2's headset path: connect over raw ATT, enable
every feature bit (several are still unidentified), subscribe to every
notify characteristic, and count what each one sends while the user is
silent vs. talking into a headset plugged into the controller's 3.5 mm jack.

Cues are rumble buzzes: 1 = stay silent, 2 = talk, 3 = done. Logs every
notification (time, phase, handle, hex) for later analysis.

    .venv312/bin/python tools/audio_probe.py <controller MAC> <adapter MAC> [log path]

Stop nso-gc.service first (it owns the controller's link).
"""

from __future__ import annotations

import collections
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ngc import protocol as P
from ngc.device import SwitchController

MAC = sys.argv[1]
ADAPTER = sys.argv[2]
LOG = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("audio_probe.tsv")

SILENT_S = 6.0
TALK_S = 8.0


def buzz(ctrl: SwitchController, times: int) -> None:
    for _ in range(times):
        try:
            ctrl.play_vibration_preset(P.GC_VIBRATION_PRESET_STRONG)
        except Exception as exc:  # noqa: BLE001
            print("buzz failed:", exc, flush=True)
        time.sleep(0.45)


def main() -> int:
    ctrl = SwitchController(MAC, ADAPTER)
    print("connecting...", flush=True)
    deadline = time.time() + 90
    while not ctrl.connect(timeout=6):
        if time.time() > deadline:
            print("could not connect")
            return 1
    print(f"connected (MTU {ctrl.att.mtu})", flush=True)

    ctrl._resolve_handles(use_cache=False)
    by_uuid = ctrl._by_uuid
    names = {ch.value_handle: uuid for uuid, ch in by_uuid.items()}

    lock = threading.Lock()
    phase = {"name": "setup"}
    counts: dict = collections.defaultdict(collections.Counter)
    sizes: dict = collections.defaultdict(set)
    rows: list[str] = []
    original = ctrl.att.notification_cb

    def on_notification(handle: int, data: bytes) -> None:
        with lock:
            counts[phase["name"]][handle] += 1
            sizes[handle].add(len(data))
            if handle != ctrl.h_input:  # input reports are known; keep the log small
                rows.append(f"{time.monotonic():.3f}\t{phase['name']}\t{handle:#06x}\t{data.hex()}")
        original(handle, data)

    ctrl.att.notification_cb = on_notification

    ctrl.enable_commands()
    ctrl.info = ctrl.read_controller_info()
    print("controller:", ctrl.info.name, flush=True)
    for flags in (0xFF,):
        try:
            ctrl.enable_features(flags)
            print(f"enabled features {flags:#04x}", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"enable features {flags:#04x} failed: {exc}", flush=True)

    notify = [ch for ch in by_uuid.values() if ch.cccd_handle]
    for ch in notify:
        try:
            ctrl.att.subscribe(ch.cccd_handle, True)
            print(f"subscribed {ch.uuid} (val {ch.value_handle:#06x})", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"subscribe {ch.uuid} failed: {exc}", flush=True)

    time.sleep(1.0)
    for name, seconds, cue in (("silent", SILENT_S, 1), ("talk", TALK_S, 2)):
        buzz(ctrl, cue)
        with lock:
            phase["name"] = name
        print(f"phase {name} ({seconds:.0f}s)", flush=True)
        time.sleep(seconds)
    with lock:
        phase["name"] = "done"
    buzz(ctrl, 3)
    ctrl.close()

    LOG.write_text("# t\tphase\thandle\thex\n" + "\n".join(rows) + "\n")
    print("\nnotifications per characteristic:")
    for handle in sorted({h for c in counts.values() for h in c}):
        per = "  ".join(f"{p}={counts[p][handle]}" for p in ("setup", "silent", "talk"))
        print(f"  {handle:#06x} {names.get(handle, '?')}  sizes={sorted(sizes[handle])}  {per}")
    print(f"\nlog: {LOG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
