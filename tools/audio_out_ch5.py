#!/usr/bin/env python3
"""Headphone-output variants on 3dacbc7e-...-6f9809e8b380 ("channel 5").

Mono Opus halves there produced faint crackling in the headset before the
controller dropped the link, so this tries Opus variants on that one
characteristic, ~2 s each, cued by N short rumble pulses for variant N.
If the link drops it waits (up to 60 s) for a button press to reconnect and
moves on. Stop nso-gc.service first.

    .venv312/bin/python tools/audio_out_ch5.py <controller MAC> <adapter MAC> [variants]
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
UUID = "3dacbc7e-6955-40b5-8eaf-6f9809e8b380"
SR = 48000
SECONDS = 2.0

_lib = ctypes.CDLL(ctypes.util.find_library("opus") or "libopus.so.0")
_lib.opus_encoder_create.restype = ctypes.c_void_p
_lib.opus_encoder_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
_lib.opus_encode.restype = ctypes.c_int
_lib.opus_encode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int16), ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
_lib.opus_encoder_ctl.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]


def tone(channels: int, frame_ms: float, bitrate: int) -> list[bytes]:
    frame = int(SR * frame_ms / 1000)
    err = ctypes.c_int()
    enc = _lib.opus_encoder_create(SR, channels, 2051, ctypes.byref(err))  # CELT, low delay
    _lib.opus_encoder_ctl(enc, 4002, bitrate)
    _lib.opus_encoder_ctl(enc, 4006, 0)  # CBR
    out = ctypes.create_string_buffer(1500)
    packets, n = [], 0
    for _ in range(int(SECONDS * 1000 / frame_ms)):
        samples = []
        for i in range(frame):
            v = int(8000 * math.sin(2 * math.pi * 1000 * (n + i) / SR))
            samples.extend([v] * channels)
        n += frame
        pcm = (ctypes.c_int16 * len(samples))(*samples)
        size = _lib.opus_encode(enc, pcm, frame, out, 1500)
        packets.append(out.raw[:size])
    return packets


def halves(pkt: bytes) -> list[bytes]:
    mid = len(pkt) // 2
    return [pkt[:mid], pkt[mid:]]


# name -> (packets, period_s, packet -> list of writes)
def variants() -> dict:
    mono20 = tone(1, 20, 40000)
    stereo20 = tone(2, 20, 40000)
    stereo20_big = tone(2, 20, 64000)
    mono10 = tone(1, 10, 40000)
    seq = {"n": 0}

    def with_seq(pkt: bytes) -> list[bytes]:
        out = []
        for h in halves(pkt):
            out.append(bytes([seq["n"] & 0xFF]) + h)
            seq["n"] += 1
        return out

    return {
        1: ("stereo 20 ms 100 B, whole", stereo20, 0.020, lambda p: [p]),
        2: ("stereo 20 ms 100 B, halves", stereo20, 0.020, halves),
        3: ("stereo 20 ms 160 B, whole", stereo20_big, 0.020, lambda p: [p]),
        4: ("mono 10 ms 50 B, whole", mono10, 0.010, lambda p: [p]),
        5: ("mono 20 ms, halves + sequence byte", mono20, 0.020, with_seq),
        6: ("mono 20 ms, halves at 7.5 ms spacing", mono20, 0.020, halves),
    }


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


def pulses(ctrl: SwitchController, times: int) -> None:
    time.sleep(0.6)
    for _ in range(times):
        ctrl.set_rumble(0.8, 0.0)
        time.sleep(0.22)
        ctrl.set_rumble(0.0, 0.0)
        time.sleep(0.5)
    time.sleep(0.6)


def main() -> int:
    table = variants()
    order = [int(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 else sorted(table)
    for k in order:
        print(f"variant {k}: {table[k][0]} ({len(table[k][1])} packets, sizes {sorted(set(map(len, table[k][1])))})")
    print("Hold Sync on the Pro Controller 2... connecting", flush=True)
    ctrl = connect(90)
    if ctrl is None:
        print("could not connect")
        return 1
    handle = ctrl._by_uuid[UUID].value_handle
    for k in order:
        if not ctrl.is_connected:
            print("  reconnecting - press a button on the Pro 2", flush=True)
            ctrl = connect(60)
            if ctrl is None:
                print("could not reconnect; stopping")
                return 1
            time.sleep(1.0)
        name, packets, period, frame = table[k]
        pulses(ctrl, k)
        print(f"[{k}] {name}", flush=True)
        t0 = time.monotonic()
        dropped = False
        for i, pkt in enumerate(packets):
            writes = frame(pkt)
            try:
                for j, w in enumerate(writes):
                    ctrl.att.write_command(handle, w)
                    if k == 6 and j + 1 < len(writes):
                        time.sleep(0.0075)
            except Exception as exc:  # noqa: BLE001
                print(f"    LINK DROPPED after {time.monotonic() - t0:.2f}s ({exc})", flush=True)
                dropped = True
                break
            wait = t0 + (i + 1) * period - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        if not dropped:
            print(f"    ok, {len(packets)} packets sent", flush=True)
    if ctrl.is_connected:
        pulses(ctrl, 8)
        ctrl.close()
    print("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
