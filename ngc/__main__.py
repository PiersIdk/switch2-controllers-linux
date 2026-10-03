"""Entry point: pair (discover + save) and run (bridge) commands.

    python -m ngc pair     # scan for a controller in pairing mode, save its address
    python -m ngc run      # run the bridge (virtual gamepad + auto-reconnect)
    python -m ngc          # run; if unconfigured, pair first
    python -m ngc joycons split|join   # separate / rejoin a connected Joy-Con pair
    python -m ngc headset on|off|toggle|status   # Pro Controller 2 headset mode
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import sys
import time

from . import protocol as P
from .config import Config


def _setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def _bond_sync(mac: str, player: int, adapter: str) -> bool:
    """Connect over raw ATT and run the bonding handshake so the controller
    reconnects automatically afterward."""
    from .bridge import prepare_bluez
    from .device import SwitchController

    prepare_bluez(mac, remove=True)
    ctrl = SwitchController(mac, adapter)
    connected = False
    for _ in range(15):
        if ctrl.connect(timeout=6):
            connected = True
            break
    if not connected:
        print("Bonding: could not establish raw link.")
        return False
    try:
        ctrl.initialize(player=player)
        ctrl.bond()
        time.sleep(1.5)
        print(f"Bonded {ctrl.info.name} ({mac}) as player {player} to adapter {adapter}.")
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"Bonding failed: {exc}")
        return False
    finally:
        ctrl.close()


async def _pair(cfg: Config, timeout: float, player: int | None = None) -> bool:
    from .scanner import find_first

    print(
        "Put a Switch 2 controller (GameCube / Pro Controller 2 / Joy-Con 2) "
        "in pairing mode (hold sync until LEDs sweep)..."
    )
    found = await find_first(timeout=timeout, require_pairing=True)
    if not found:
        print("No controller found in pairing mode.")
        return False
    if not cfg.adapter_mac:
        from .config import detect_adapter

        cfg.adapter_mac = detect_adapter()
    entry = cfg.add_controller(found.device.address, name=found.name, player=player)
    cfg.save()
    print(f"Discovered {found.name} at {entry.mac} (player {entry.player}); bonding...")
    ok = await asyncio.get_event_loop().run_in_executor(
        None, _bond_sync, entry.mac, entry.player, cfg.adapter_mac
    )
    if ok:
        cfg.mark_bonded(entry.mac, True)
        cfg.save()
    return ok


async def _rebond(cfg: Config, timeout: float) -> bool:
    """Re-run the bonding handshake for a controller already in config."""
    from .scanner import find_first

    print(
        "Hold Sync on the controller until the LEDs sweep (pairing mode), "
        "then release — this re-bonds it for button-wake reconnect."
    )
    found = await find_first(timeout=timeout, require_pairing=True)
    if not found:
        print("No controller found in pairing mode.")
        return False
    macs = {e.mac.upper() for e in cfg.entries()}
    if found.device.address.upper() not in macs:
        print(f"{found.device.address} is not in your saved list — use 'pair' to add it.")
        return False
    if not cfg.adapter_mac:
        from .config import detect_adapter

        cfg.adapter_mac = detect_adapter()
    entry = cfg.add_controller(found.device.address, name=found.name)
    cfg.save()
    cfg.mark_bonded(found.device.address, False)
    cfg.save()
    print(f"Re-bonding {found.name} at {entry.mac} (player {entry.player})...")
    ok = await asyncio.get_event_loop().run_in_executor(
        None, _bond_sync, entry.mac, entry.player, cfg.adapter_mac
    )
    if ok:
        cfg.mark_bonded(entry.mac, True)
        cfg.save()
    return ok


def _list(cfg: Config) -> int:
    entries = cfg.entries()
    if not entries:
        print("No controllers configured. Run: python -m ngc pair")
        return 0
    print(f"Adapter: {cfg.adapter_mac}")
    for e in entries:
        label = e.name or "Switch 2 Controller"
        bond = "bonded" if e.bonded else "needs bond (connect once with Sync)"
        print(f"  P{e.player}  {e.mac}  {label}  [{bond}]")
    return 0


def _remove(cfg: Config, mac: str) -> int:
    if not cfg.remove_controller(mac):
        print(f"Controller {mac} is not in your saved list.")
        return 1
    cfg.save()
    print(f"Removed {mac.upper()}. Restart the bridge to apply.")
    return 0


def _swap(cfg: Config, player_a: int, player_b: int) -> int:
    ca = cfg.find_by_player(player_a)
    cb = cfg.find_by_player(player_b)
    if not ca or not cb:
        print(f"Could not swap player {player_a} and {player_b} — check both are saved.")
        return 1
    if not cfg.swap_players(player_a, player_b):
        return 1
    cfg.save()
    la = ca.name or ca.mac
    lb = cb.name or cb.mac
    print(f"Swapped: {la} is now P{player_b}, {lb} is now P{player_a}. Restart the bridge to apply.")
    return 0


def _joycons(action: str | None) -> int:
    """Ask the running bridge to split or rejoin the Joy-Con pair. The bridge
    owns the links, so this just leaves a request it picks up within ~1.5 s
    (see Bridge._check_joycon_request)."""
    from .bridge import JOYCON_REQUEST_PATH

    if action not in ("split", "join"):
        print("Usage: ngc joycons split|join")
        return 1
    JOYCON_REQUEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    JOYCON_REQUEST_PATH.write_text(action)
    print(f"Requested Joy-Con {action}.")
    return 0


def _headset(cfg: Config, action: str | None) -> int:
    """Pro Controller 2 headset mode: the headset in its 3.5 mm jack becomes a
    microphone, but motion and battery aren't decoded while it's on."""
    from .bridge import HEADSET_REQUEST_PATH

    current = cfg.pro2_headset_mic
    if action in (None, "status"):
        print(f"Headset mode is {'on' if current else 'off'}.")
        return 0
    if action not in ("on", "off", "toggle"):
        print("Usage: ngc headset on|off|toggle|status")
        return 1
    want = (not current) if action == "toggle" else (action == "on")
    HEADSET_REQUEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    HEADSET_REQUEST_PATH.write_text("on" if want else "off")
    print(f"Headset mode {'on (mic on, motion off)' if want else 'off (motion on)'}.")
    return 0


def _run(cfg: Config) -> int:
    from .bridge import Bridge

    bridge = Bridge(cfg)

    def _sig(_signum, _frame):
        bridge.stop()

    def _usr1(_signum, _frame):
        # Dolphin: Save to Selected Slot = Back + Shoulder R (C + R on ngc GC pad).
        try:
            bridge.pulse_gamecube_hotkey("C", "R")
        except Exception as exc:  # noqa: BLE001
            logging.getLogger(__name__).warning("hotkey pulse failed: %s", exc)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGUSR1, _usr1)
    bridge.run()
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="ngc", description="Switch 2 controller bridge (GameCube / Pro Controller 2 / Joy-Con 2)")
    parser.add_argument("command", nargs="?", default="run",
                        choices=["run", "pair", "rebond", "list", "remove", "swap", "joycons", "headset"])
    parser.add_argument("action", nargs="?", choices=["split", "join", "on", "off", "toggle", "status"],
                        help="joycons: split|join; headset: on|off|toggle|status")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--timeout", type=float, default=30.0, help="pairing scan timeout")
    parser.add_argument("--mac", help="controller MAC (for remove)")
    parser.add_argument("--player", type=int, help="player slot 1-8 (for pair)")
    parser.add_argument("--players", nargs=2, type=int, metavar=("A", "B"), help="player slots to swap")
    args = parser.parse_args(argv)
    _setup_logging(args.verbose)

    cfg = Config.load()

    if args.command == "pair":
        return 0 if asyncio.run(_pair(cfg, args.timeout, args.player)) else 1

    if args.command == "rebond":
        return 0 if asyncio.run(_rebond(cfg, args.timeout)) else 1

    if args.command == "remove":
        if not args.mac:
            print("Usage: ngc remove --mac AA:BB:CC:DD:EE:FF")
            return 1
        return _remove(cfg, args.mac)

    if args.command == "swap":
        a, b = (args.players if args.players else (1, 2))
        return _swap(cfg, a, b)

    if args.command == "list":
        return _list(cfg)

    if args.command == "joycons":
        return _joycons(args.action)

    if args.command == "headset":
        return _headset(cfg, args.action)

    if not cfg.entries():
        print("No controller configured; scanning for one in pairing mode first.")
        if not asyncio.run(_pair(cfg, args.timeout)):
            return 1
    return _run(cfg)


if __name__ == "__main__":
    sys.exit(main())
