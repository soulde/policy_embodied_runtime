from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

from .assets import DEFAULT_SCENE
from .serial_bus import St3215SerialBus
from .sim import SoArm101Sim
from .virtual_serial import VirtualSerialEndpoint

VIRTUAL_SERIAL_PATH = Path("/tmp/rusty_robot_soarm101")


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-ARM101 MuJoCo ST3215 serial simulator")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--hz", type=float, default=200.0)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()

    sim = SoArm101Sim(args.scene)
    period = 1.0 / args.hz

    with VirtualSerialEndpoint.open() as serial:
        create_symlink(VIRTUAL_SERIAL_PATH, serial.path)
        try:
            print(f"{VIRTUAL_SERIAL_PATH} -> {serial.path}", flush=True)
            bus = St3215SerialBus(serial, sim)
            try:
                if args.gui:
                    run_gui_loop(sim, bus, period)
                else:
                    run_loop(sim, bus, period)
            except KeyboardInterrupt:
                pass
        finally:
            remove_symlink(VIRTUAL_SERIAL_PATH, serial.path)


def run_loop(sim: SoArm101Sim, bus: St3215SerialBus, period: float) -> None:
    next_step = time.monotonic()
    try:
        while True:
            bus.poll()
            sim.step()
            next_step += period
            sleep_for = next_step - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_step = time.monotonic()
    except KeyboardInterrupt:
        pass


def run_gui_loop(sim: SoArm101Sim, bus: St3215SerialBus, period: float) -> None:
    import mujoco.viewer

    next_step = time.monotonic()
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                bus.poll()
                sim.step()
                viewer.sync()
                next_step += period
                sleep_for = next_step - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_step = time.monotonic()
    except KeyboardInterrupt:
        pass


def create_symlink(link: Path, target: str) -> None:
    link.parent.mkdir(parents=True, exist_ok=True)
    try:
        link.unlink()
    except FileNotFoundError:
        pass
    os.symlink(target, link)


def remove_symlink(link: Path, target: str) -> None:
    try:
        if link.is_symlink() and os.readlink(link) == target:
            link.unlink()
    except FileNotFoundError:
        pass


if __name__ == "__main__":
    main()
