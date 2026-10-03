#!/usr/bin/env python3
"""Decode a Pro Controller 2 headset-mic recording (an audio_probe*.py log)
to a WAV file. See docs/headset-audio.md for the stream format.

    python3 tools/decode_mic.py audio_probe3.tsv mic.wav
"""

from __future__ import annotations

import array
import ctypes
import sys
import wave

MIC_HANDLE = "0x002e"  # 7492866c-ec3e-4619-8258-32755ffcc0f9 on the Pro 2 tested
SAMPLE_RATE = 48000


def main(src: str, out: str) -> int:
    opus = ctypes.CDLL("libopus.so.0")
    opus.opus_decoder_create.restype = ctypes.c_void_p
    opus.opus_decoder_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    opus.opus_decode.restype = ctypes.c_int
    opus.opus_decode.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int,
                                 ctypes.POINTER(ctypes.c_int16), ctypes.c_int, ctypes.c_int]
    err = ctypes.c_int()
    dec = opus.opus_decoder_create(SAMPLE_RATE, 1, ctypes.byref(err))
    pcm = array.array("h")
    packet = None

    def flush(data: bytes) -> None:
        buf = (ctypes.c_int16 * 5760)()
        n = opus.opus_decode(dec, data, len(data), buf, 5760, 0)
        if n > 0:
            pcm.extend(buf[:n])

    for line in open(src):
        if line.startswith("#"):
            continue
        _t, _phase, handle, hexdata = line.rstrip("\n").split("\t")
        if handle != MIC_HANDLE:
            continue
        report = bytes.fromhex(hexdata)
        if report[14] != 50:          # no mic fragment in this report
            continue
        fragment = report[15:65]
        if report[13] == 0x0F:        # first fragment of a packet
            if packet:
                flush(bytes(packet))
            packet = bytearray(fragment)
        elif report[13] == 0x07 and packet is not None:  # continuation
            packet += fragment
    if packet:
        flush(bytes(packet))

    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm.tobytes())
    print(f"wrote {out}: {len(pcm) / SAMPLE_RATE:.1f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
