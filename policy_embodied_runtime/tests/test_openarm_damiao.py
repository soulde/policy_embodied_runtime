from policy_embodied_runtime.sim.openarm.damiao import (
    DISABLE,
    ENABLE,
    ZERO_POSITION,
    Limits,
    decode_mit,
    encode_feedback,
)


def _encode(value: float, minimum: float, maximum: float, maximum_code: int) -> int:
    return int((value - minimum) / (maximum - minimum) * maximum_code + 0.5)


def test_control_frames_match_damiao_special_commands() -> None:
    assert ENABLE == bytes.fromhex("ff ff ff ff ff ff ff fc")
    assert DISABLE == bytes.fromhex("ff ff ff ff ff ff ff fd")
    assert ZERO_POSITION == bytes.fromhex("ff ff ff ff ff ff ff fe")


def test_decodes_native_mit_layout() -> None:
    limits = Limits()
    p = _encode(1.25, -limits.position, limits.position, 0xFFFF)
    v = _encode(-2.0, -limits.velocity, limits.velocity, 0xFFF)
    kp = _encode(20.0, 0.0, 500.0, 0xFFF)
    kd = _encode(0.5, 0.0, 5.0, 0xFFF)
    torque = _encode(1.0, -limits.torque, limits.torque, 0xFFF)
    payload = bytes(
        (
            p >> 8,
            p & 0xFF,
            v >> 4,
            (v & 0xF) << 4 | kp >> 8,
            kp & 0xFF,
            kd >> 4,
            (kd & 0xF) << 4 | torque >> 8,
            torque & 0xFF,
        )
    )
    command = decode_mit(payload, limits)
    assert abs(command.position - 1.25) < 0.001
    assert abs(command.velocity + 2.0) < 0.02
    assert abs(command.kp - 20.0) < 0.13
    assert abs(command.kd - 0.5) < 0.002
    assert abs(command.torque - 1.0) < 0.01


def test_feedback_uses_native_damiao_layout() -> None:
    payload = encode_feedback(3, 1.0, 2.0, 3.0, Limits())
    assert len(payload) == 8
    assert payload[0] == 3
    assert payload[6:] == bytes((25, 25))
