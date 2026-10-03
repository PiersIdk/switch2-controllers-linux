"""Pro Controller 2 headset mic -> a PipeWire / PulseAudio microphone.

The mic of a headset in the controller's 3.5 mm jack arrives inside the
extended 112-byte input reports (see docs/headset-audio.md): Opus, 48 kHz
mono, 20 ms per 100-byte packet, split into two 50-byte fragments. This
reassembles the packets, decodes them with libopus and writes the PCM into a
pipe that a `module-pipe-source` virtual microphone reads, so any app
(Discord, OBS, games) can pick "Pro Controller 2 Headset Mic" as its input.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import errno
import logging
import os
import subprocess
import tempfile
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

SAMPLE_RATE = 48000
MAX_FRAME = 5760  # 120 ms at 48 kHz, libopus's largest frame

FRAGMENT_START = 0x0F
FRAGMENT_CONTINUE = 0x07
FRAGMENT_LEN = 50


def _load_opus():
    name = ctypes.util.find_library("opus") or "libopus.so.0"
    lib = ctypes.CDLL(name)
    lib.opus_decoder_create.restype = ctypes.c_void_p
    lib.opus_decoder_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    lib.opus_decode.restype = ctypes.c_int
    lib.opus_decode.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int,
                                ctypes.POINTER(ctypes.c_int16), ctypes.c_int, ctypes.c_int]
    lib.opus_decoder_destroy.argtypes = [ctypes.c_void_p]
    return lib


class VirtualMic:
    """A module-pipe-source microphone fed with 16-bit 48 kHz mono PCM."""

    def __init__(self, source_name: str, description: str) -> None:
        self._dir = tempfile.mkdtemp(prefix="ngc-mic-")
        self.fifo = os.path.join(self._dir, "pcm")
        self.module_id: Optional[str] = None
        self._fd: Optional[int] = None
        desc = description.replace(" ", "\\ ")
        out = subprocess.run(
            ["pactl", "load-module", "module-pipe-source",
             f"source_name={source_name}", f"file={self.fifo}",
             "format=s16le", f"rate={SAMPLE_RATE}", "channels=1",
             f"source_properties=device.description={desc}"],
            capture_output=True, text=True, timeout=5,
        )
        if out.returncode != 0:
            raise RuntimeError(out.stderr.strip() or "pactl load-module failed")
        self.module_id = out.stdout.strip()
        logger.info("virtual microphone ready: %s (module %s)", description, self.module_id)

    def write(self, pcm: bytes) -> None:
        if self._fd is None:
            try:
                self._fd = os.open(self.fifo, os.O_WRONLY | os.O_NONBLOCK)
            except OSError:
                return  # nothing reading the pipe yet
        try:
            os.write(self._fd, pcm)
        except BlockingIOError:
            pass  # reader behind; drop this frame rather than stall input
        except OSError as exc:
            if exc.errno == errno.EPIPE:
                os.close(self._fd)
                self._fd = None

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        if self.module_id:
            subprocess.run(["pactl", "unload-module", self.module_id],
                           capture_output=True, timeout=5)
            self.module_id = None
        try:
            Path(self.fifo).unlink(missing_ok=True)
            os.rmdir(self._dir)
        except OSError:
            pass


class HeadsetMic:
    """Reassembles and decodes one controller's mic fragments into a VirtualMic."""

    def __init__(self, mic: VirtualMic) -> None:
        self._opus = _load_opus()
        err = ctypes.c_int()
        self._dec = self._opus.opus_decoder_create(SAMPLE_RATE, 1, ctypes.byref(err))
        if not self._dec:
            raise RuntimeError(f"opus_decoder_create failed ({err.value})")
        self._mic = mic
        self._packet: Optional[bytearray] = None
        self._pcm = (ctypes.c_int16 * MAX_FRAME)()
        self._lock = threading.Lock()

    def feed(self, report: bytes) -> None:
        """Feed one 112-byte extended report (bytes 13-64 carry the mic)."""
        if len(report) < 15 + FRAGMENT_LEN or report[14] != FRAGMENT_LEN:
            return
        fragment = report[15:15 + FRAGMENT_LEN]
        with self._lock:
            if report[13] == FRAGMENT_START:
                self._flush()
                self._packet = bytearray(fragment)
            elif report[13] == FRAGMENT_CONTINUE and self._packet is not None:
                self._packet += fragment
                self._flush()

    def _flush(self) -> None:
        packet, self._packet = self._packet, None
        if not packet:
            return
        n = self._opus.opus_decode(self._dec, bytes(packet), len(packet), self._pcm, MAX_FRAME, 0)
        if n > 0:
            self._mic.write(ctypes.string_at(self._pcm, n * 2))

    def close(self) -> None:
        with self._lock:
            if self._dec:
                self._opus.opus_decoder_destroy(self._dec)
                self._dec = None
