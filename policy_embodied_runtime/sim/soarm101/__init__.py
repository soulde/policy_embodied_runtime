"""SO-ARM101 MuJoCo simulator with an ST3215 virtual serial bus."""

__all__ = ["SoArm101Sim", "VirtualSerialEndpoint"]


def __getattr__(name: str) -> object:
    """Lazily import simulator objects so MuJoCo remains an optional dependency."""
    if name == "SoArm101Sim":
        from policy_embodied_runtime.sim.soarm101.sim import SoArm101Sim

        return SoArm101Sim
    if name == "VirtualSerialEndpoint":
        from policy_embodied_runtime.sim.soarm101.virtual_serial import VirtualSerialEndpoint

        return VirtualSerialEndpoint
    raise AttributeError(name)
