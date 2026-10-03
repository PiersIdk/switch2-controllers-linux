#!/usr/bin/env python3
"""Map the Pro Controller 2's extended (headset-capable) input report.

Records a few seconds of the normal 63-byte reports (ab7de9be-...-fd2) for
reference, then switches the controller to the 112-byte reports on
7492866c-...-f9 (all feature bits on) and walks through timed prompts -
each button, both sticks, a tilt - logging every report with the prompt it
was captured under. Stop nso-gc.service first.

    .venv312/bin/python tools/pro2_ext_capture.py <controller MAC> <adapter MAC> [log]
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ngc.device import SwitchController

MAC, ADAPTER = sys.argv[1], sys.argv[2]
LOG = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("pro2_ext_capture.tsv")
EXT_UUID = "7492866c-ec3e-4619-8258-32755ffcc0f9"

PROMPTS = [
    ("idle", "Put it down flat and still"),
    ("A", "Hold A"), ("B", "Hold B"), ("X", "Hold X"), ("Y", "Hold Y"),
    ("L", "Hold L"), ("R", "Hold R"), ("ZL", "Hold ZL"), ("ZR", "Hold ZR"),
    ("MINUS", "Hold -"), ("PLUS", "Hold +"), ("HOME", "Hold Home"),
    ("CAPTURE", "Hold Capture"), ("C", "Hold C (the GameChat button)"),
    ("GL", "Hold GL (back paddle, left)"), ("GR", "Hold GR (back paddle, right)"),
    ("L_STK", "Click the left stick in"), ("R_STK", "Click the right stick in"),
    ("UP", "Hold d-pad Up"), ("DOWN", "Hold d-pad Down"),
    ("LEFT", "Hold d-pad Left"), ("RIGHT", "Hold d-pad Right"),
    ("lstick", "Roll the LEFT stick around its edge"),
    ("rstick", "Roll the RIGHT stick around its edge"),
    ("tilt", "Pick it up and tilt it in every direction"),
]
PROMPT_S = 3.5
GAP_S = 1.5


def say(msg: str) -> None:
    print(msg, flush=True)


def main() -> int:
    ctrl = SwitchController(MAC, ADAPTER)
    say("Hold Sync on the Pro Controller 2 now... connecting")
    deadline = time.time() + 90
    while not ctrl.connect(timeout=6):
        if time.time() > deadline:
            say("could not connect")
            return 1
    ctrl._resolve_handles(use_cache=False)
    ext = ctrl._by_uuid[EXT_UUID]

    lock = threading.Lock()
    state = {"phase": "normal"}
    rows: list[str] = []
    original = ctrl.att.notification_cb

    def on_notification(handle: int, data: bytes) -> None:
        if handle in (ctrl.h_input, ext.value_handle):
            with lock:
                rows.append(f"{time.monotonic():.3f}\t{state['phase']}\t{handle:#06x}\t{data.hex()}")
        original(handle, data)

    ctrl.att.notification_cb = on_notification
    ctrl.enable_commands()
    ctrl.info = ctrl.read_controller_info()
    ctrl._read_calibration()

    # Reference: the normal format the bridge already decodes.
    ctrl.enable_features(0x03 | 0x04)
    ctrl.att.subscribe(ctrl.h_input_cccd, True)
    say("Recording the normal format - put it down flat and still (4s)")
    time.sleep(4.0)

    # Extended format.
    with lock:
        state["phase"] = "switching"
    ctrl.att.subscribe(ext.cccd_handle, True)
    ctrl.enable_features(0xFF)
    time.sleep(1.0)
    say(f"\nNow {len(PROMPTS)} prompts, {PROMPT_S:.1f}s each. Hold the controller normally.\n")
    time.sleep(1.5)
    for i, (label, prompt) in enumerate(PROMPTS, 1):
        with lock:
            state["phase"] = "release"
        say(f"[{i}/{len(PROMPTS)}] next: {prompt}")
        time.sleep(GAP_S)
        with lock:
            state["phase"] = label
        say("    >>> NOW")
        time.sleep(PROMPT_S)
    with lock:
        state["phase"] = "done"
    say("\nDone - let go.")
    ctrl.close()
    header = [f"# calib_L={ctrl.left_calib} calib_R={ctrl.right_calib}",
              f"# normal_handle={ctrl.h_input:#06x} ext_handle={ext.value_handle:#06x}",
              "# t\tphase\thandle\thex"]
    LOG.write_text("\n".join(header + rows) + "\n")
    say(f"Saved {len(rows)} reports to {LOG}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
