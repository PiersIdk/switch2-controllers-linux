# Pro Controller 2 headset (3.5 mm jack) — what's known

Found by probing a Pro Controller 2 over the raw ATT link with a 4-pole
(CTIA) headset plugged in (tools/audio_probe*.py, tools/decode_mic.py).
Not standard Bluetooth LE Audio: the controller exposes no LE Audio services
(no 0x1850 PACS / 0x184E ASCS) — everything is in Nintendo's own service.

## Extended input reports

Enabling every feature bit (`enable_features(0xFF)`) switches input from the
63-byte reports on `ab7de9be-…-fd2` to 112-byte reports on
`7492866c-ec3e-4619-8258-32755ffcc0f9` (~133/s, the 7.5 ms link interval).
Which single bit does it is not pinned down yet: once switched, the
controller stayed in this mode for the rest of the session. Byte 11 reads
0x30, or 0x38 once feature bit 0x20 is set.

| Bytes  | Meaning |
|--------|---------|
| 13     | headset / mic fragment status: `0x00` no headset, `0x0F` first fragment of a mic packet, `0x07` continuation |
| 14     | mic fragment length — `0x32` (50) when present, `0x00` otherwise |
| 15–64  | mic fragment (50 bytes) |
| 65+    | other controller data (motion etc., not yet mapped) |

## Headset mic: Opus

Each mic packet is **Opus, 48 kHz mono, 20 ms, 100 bytes** (TOC byte `0xF8`:
CELT-only fullband, 20 ms, one frame), split into two 50-byte fragments
(byte 13 = `0x0F`, then `0x07`). Verified: all packets decode with libopus
with no errors, and a 1 kHz tone played into the mic shows up ~150× above
the silent level, gone again when stopped. Unplugging the headset stops
the fragments (byte 13 → `0x00`).

## Headphone output

Not found. A 1 kHz tone encoded exactly like the mic stream (Opus, 48 kHz
mono, 20 ms CBR 100-byte packets, TOC 0xF8) was streamed every 20 ms to
each unused write-without-response characteristic, as whole packets, two
50-byte halves, and halves with the mic's 0x0F/0x07 + 0x32 header
(tools/audio_out_probe.py), with feature bits 0xFF on:

| Characteristic             | Result |
|----------------------------|--------|
| `3dacbc7e-…-6f9809e8b379`  | tone plays through the **rumble actuator** |
| `4147423d-…-d23e5df59f8d`  | accepted, no audible effect |
| `ab7de9be-…-118f09df7fdf`  | controller drops the link (whole packet and mic-header halves) |
| `cc483f51-…-630c31f72b06`  | tone plays through the **rumble actuator** |
| `3dacbc7e-…-6f9809e8b380`  | controller drops the link (whole packet and plain halves) |

Follow-up on `3dacbc7e-…-6f9809e8b380` (tools/audio_out_ch5.py): mono halves
and stereo 20 ms whole packets both produced **faint crackling in the
headset** before the drop, so this is very likely the headphone output.
Every variant tried — stereo 100 B whole / halves, stereo 160 B whole, mono
10 ms 50 B — made the controller drop the link within 0.4–1.2 s. Likely the
console sends a "start headphone audio" command first and the controller
treats an unannounced stream as an error. Not brute-forced: some known
commands write controller memory or pairing, so guessing command IDs risks
calibration or pairing data.

Nothing reached the headphones cleanly, so the jack's output path presumably needs
a setup command from the host first (route to jack / volume / open stream)
or a framing not tried here. Next step: sniff a Switch 2 console talking to
the controller while a game plays sound (e.g. an nRF52840 BLE sniffer).

Note for cues: on the Pro 2, vibration preset 0x02 is the "find
controller" chime; use short HD-rumble pulses (set_rumble) for countable
cues.

## Headset mode in the bridge

`pro2_headset_mic` (config) / `python -m ngc headset on|off|toggle|status`
(or `scripts/pro2-headset.sh`) switches a connected Pro 2 live between the
normal reports and the extended ones. In headset mode the mic is decoded
into a PipeWire source ("Pro Controller 2 Headset Mic", via
module-pipe-source); buttons and sticks come from the extended report, but
motion and battery aren't decoded there yet (the block after the mic is a
packed, length-prefixed stream).
