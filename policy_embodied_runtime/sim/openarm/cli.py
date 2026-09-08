from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco.viewer

from .sim import DEFAULT_SCENE, OpenArmSim
from .socketcan_bus import DamiaoSocketCanBus


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="Expose OpenArm MuJoCo as seven Damiao motors on SocketCAN"
    )
    value.add_argument("--interface", default="vcan0")
    value.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    value.add_argument("--headless", action="store_true")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    sim = OpenArmSim(args.scene)
    bus = DamiaoSocketCanBus(args.interface, sim)
    viewer = None if args.headless else mujoco.viewer.launch_passive(sim.model, sim.data)
    try:
        while viewer is None or viewer.is_running():
            started = time.monotonic()
            bus.poll()
            sim.step()
            if viewer is not None:
                viewer.sync()
            remaining = sim.model.opt.timestep - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        return 0
    finally:
        bus.close()
        if viewer is not None:
            viewer.close()
    return 0
