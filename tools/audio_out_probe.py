#!/usr/bin/env python3
"""Look for the Pro Controller 2's headphone-output path.

Encodes a 1 kHz tone in the format the headset mic uses (Opus, 48 kHz mono,
20 ms CBR 100-byte packets) and streams it, every 20 ms, to each unused
write-without-response characteristic in Nintendo's service in turn, trying
a few plausible framings on each. Before each characteristic the controller
buzzes N times (N = 1..5) so the listener can say which one made a sound.
Stop nso-gc.service first.

    .venv312/bin/python tools/audio_out_probe.py <controller MAC> <adapter MAC>
"""

from __future__ import annotations

import ctypes
import ctypes.util
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ngc import protocol as P
from ngc.device import SwitchController

MAC, ADAPTER = sys.argv[1], sys.argv[2]
CANDIDATES = [
    "3dacbc7e-6955-40b5-8eaf-6f9809e8b379",
    "4147423d-fdae-4df7-a4f7-d23e5df59f8d",
    "ab7de9be-89fe-49ad-828f-118f09df7fdf",
    "cc483f51-9258-427d-a939-630c31f72b06",
    "3dacbc7e-6955-40b5-8eaf-6f9809e8b380",
]
FRAMING_S = 2.0
ORDER = [int(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 else [3, 4, 5, 1, 2]
# Optional 4th arg: which layouts, in order (whole, halves, mic).
LAYOUTS = sys.argv[4].split(",") if len(sys.argv) > 4 else ["whole", "halves", "mic"]
_LAYOUT_NAMES = {"whole": "whole packet", "halves": "two halves", "mic": "halves with mic header"}
SR, FRAME = 48000, 960  # 20 ms


def opus_tone_packets(seconds: float) -> list[bytes]:
    lib = ctypes.CDLL(ctypes.util.find_library("opus") or "libopus.so.0")
    lib.opus_encoder_create.restype = ctypes.c_void_p
    lib.opus_encoder_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    lib.opus_encode.restype = ctypes.c_int
    lib.opus_encode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int16), ctypes.c_int,
                                ctypes.c_char_p, ctypes.c_int]
    err = ctypes.c_int()
    enc = lib.opus_encoder_create(SR, 1, 2051, ctypes.byref(err))  # RESTRICTED_LOWDELAY = CELT
    lib.opus_encoder_ctl.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.opus_encoder_ctl(enc, 4002, 40000)  # OPUS_SET_BITRATE: 40 kbit/s = 100 bytes / 20 ms
    lib.opus_encoder_ctl(enc, 4006, 0)      # OPUS_SET_VBR off
    packets, n = [], 0
    out = ctypes.create_string_buffer(400)
    for _ in range(int(seconds * SR / FRAME)):
        pcm = (ctypes.c_int16 * FRAME)(*[int(8000 * math.sin(2 * math.pi * 1000 * (n + i) / SR)) for i in range(FRAME)])
        n += FRAME
        size = lib.opus_encode(enc, pcm, FRAME, out, 400)
        packets.append(out.raw[:size])
    return packets


def framings(pkt: bytes, seq: int) -> dict:
    halves = [pkt[:50], pkt[50:100]]
    return {
        "whole packet": [pkt],
        "two halves": halves,
        "halves with mic header": [bytes([0x0F, 0x32]) + halves[0], bytes([0x07, 0x32]) + halves[1]],
    }


def buzz(ctrl: SwitchController, times: int) -> None:
    """Countable cue: short plain rumble pulses. (The vibration preset used
    before is, on the Pro 2, the "find controller" chime - several overlap
    and can't be counted.)"""
    time.sleep(0.6)
    for _ in range(times):
        ctrl.set_rumble(0.8, 0.0)
        time.sleep(0.22)
        ctrl.set_rumble(0.0, 0.0)
        time.sleep(0.5)


def connect(wait_s: float) -> SwitchController | None:
    ctrl = SwitchController(MAC, ADAPTER)
    deadline = time.time() + wait_s
    while not ctrl.connect(timeout=6):
        if time.time() > deadline:
            return None
    ctrl._resolve_handles(use_cache=False)
    ctrl.enable_commands()
    ctrl.info = ctrl.read_controller_info()
    ctrl._resolve_vibration_handle()
    ctrl._start_hd_worker()
    ext = ctrl._by_uuid.get(P.EXTENDED_INPUT_UUID)
    if ext is not None:
        ctrl.att.subscribe(ext.cccd_handle, True)
    ctrl.enable_features(0xFF)
    return ctrl


def main() -> int:
    packets = opus_tone_packets(FRAMING_S)
    print(f"tone: {len(packets)} Opus packets, sizes {sorted(set(map(len, packets)))}", flush=True)
    print("Hold Sync on the Pro Controller 2... connecting", flush=True)
    ctrl = connect(90)
    if ctrl is None:
        print("could not connect")
        return 1
    print("connected; starting in 2s", flush=True)
    time.sleep(2.0)

    # Untested-first order (a first run dropped the link during channel 2).
    for n in ORDER:
        if ctrl is None or not ctrl.is_connected:
            print("reconnecting (press a button on the Pro 2 if it doesn't come back)...", flush=True)
            ctrl = connect(45)
            if ctrl is None:
                print("could not reconnect; stopping")
                return 1
            time.sleep(1.5)
        uuid = CANDIDATES[n - 1]
        ch = ctrl._by_uuid.get(uuid)
        if ch is None:
            print(f"[{n}] {uuid}: not present")
            continue
        buzz(ctrl, n)
        time.sleep(0.8)
        print(f"[{n}] {uuid} (handle {ch.value_handle:#06x})", flush=True)
        for name in [_LAYOUT_NAMES[k] for k in LAYOUTS]:
            print(f"      {name}", flush=True)
            t0 = time.monotonic()
            for seq, pkt in enumerate(packets):
                try:
                    for write in framings(pkt, seq)[name]:
                        ctrl.att.write_command(ch.value_handle, write)
                except Exception as exc:  # noqa: BLE001
                    print(f"LINK DROPPED on channel {n}, layout '{name}': {exc}", flush=True)
                    break
                wait = t0 + (seq + 1) * FRAME / SR - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            if not ctrl.is_connected:
                break
        time.sleep(0.8)
    if ctrl is not None and ctrl.is_connected:
        buzz(ctrl, 6)
        ctrl.close()
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
