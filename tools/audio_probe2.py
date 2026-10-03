#!/usr/bin/env python3
"""Second headset probe for the Pro Controller 2.

1. Enables the known base features plus ONE still-unidentified feature bit
   at a time, to find which bit switches input to the 112-byte reports on
   7492866c-...-f9 that carry the suspected mic block (bytes 14..69).
2. With everything on, rumble-cued phases: 1 buzz silent, 2 buzzes tap the
   headset mic rhythmically, 3 buzzes unplug the headset, 4 buzzes done.

Logs every non-input notification (time, phase, handle, hex).

    .venv312/bin/python tools/audio_probe2.py <controller MAC> <adapter MAC> [log]
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

MAC, ADAPTER = sys.argv[1], sys.argv[2]
LOG = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("audio_probe2.tsv")

BASE = 0x03 | P.FEATURE_MOTION
UNKNOWN_BITS = (0x08, 0x20, 0x40, 0x80)
PHASES = (("silent", 5.0, 1), ("tap", 8.0, 2), ("unplugged", 6.0, 3))


def buzz(ctrl: SwitchController, times: int) -> None:
    for _ in range(times):
        try:
            ctrl.play_vibration_preset(P.GC_VIBRATION_PRESET_STRONG)
        except Exception:  # noqa: BLE001
            pass
        time.sleep(0.45)


def main() -> int:
    ctrl = SwitchController(MAC, ADAPTER)
    print("connecting...", flush=True)
    deadline = time.time() + 90
    while not ctrl.connect(timeout=6):
        if time.time() > deadline:
            print("could not connect")
            return 1
    ctrl._resolve_handles(use_cache=False)
    names = {ch.value_handle: uuid for uuid, ch in ctrl._by_uuid.items()}

    lock = threading.Lock()
    phase = {"name": "setup"}
    counts: dict = collections.defaultdict(collections.Counter)
    rows: list[str] = []
    original = ctrl.att.notification_cb

    def on_notification(handle: int, data: bytes) -> None:
        with lock:
            counts[phase["name"]][handle] += 1
            if handle != ctrl.h_input:
                rows.append(f"{time.monotonic():.3f}\t{phase['name']}\t{handle:#06x}\t{data.hex()}")
        original(handle, data)

    ctrl.att.notification_cb = on_notification
    ctrl.enable_commands()
    ctrl.info = ctrl.read_controller_info()
    for ch in ctrl._by_uuid.values():
        if ch.cccd_handle:
            try:
                ctrl.att.subscribe(ch.cccd_handle, True)
            except Exception:  # noqa: BLE001
                pass
    print("connected:", ctrl.info.name, flush=True)

    def set_phase(name: str) -> None:
        with lock:
            phase["name"] = name

    for bit in (0,) + UNKNOWN_BITS:
        flags = BASE | bit
        set_phase(f"bit{bit:#04x}")
        try:
            ctrl.enable_features(flags)
        except Exception as exc:  # noqa: BLE001
            print(f"features {flags:#04x} failed: {exc}", flush=True)
        time.sleep(3.0)

    ctrl.enable_features(0xFF)
    time.sleep(0.5)
    for name, seconds, cue in PHASES:
        buzz(ctrl, cue)
        set_phase(name)
        print(f"phase {name}", flush=True)
        time.sleep(seconds)
    set_phase("done")
    buzz(ctrl, 4)
    ctrl.close()

    LOG.write_text("# t\tphase\thandle\thex\n" + "\n".join(rows) + "\n")
    print("\nnotifications per phase:")
    for ph in [p for p in counts if p != "done"]:
        print(f"  {ph:12s} " + "  ".join(f"{names.get(h, hex(h))[:8]}={n}" for h, n in sorted(counts[ph].items())))
    print(f"\nlog: {LOG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
