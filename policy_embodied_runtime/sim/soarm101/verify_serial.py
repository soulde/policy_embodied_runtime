from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass

from . import st3215
from .serial_bus import St3215SerialBus
from .sim import SoArm101Sim
from .virtual_serial import VirtualSerialEndpoint


@dataclass(frozen=True)
class VerificationResult:
    serial_path: str
    start_units: int
    target_units: int
    end_units: int


def verify_serial_control_and_feedback(
    *,
    device_id: int = 1,
    target_units: int = 3072,
    steps: int = 240,
    step_period_s: float = 0.005,
) -> VerificationResult:
    sim = SoArm101Sim()

    with VirtualSerialEndpoint.open() as serial:
        bus = St3215SerialBus(serial, sim)
        client_fd = os.open(serial.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            ping(client_fd, bus, device_id)
            start_units = read_present_position(client_fd, bus, device_id)
            write_goal_position(client_fd, bus, device_id, target_units)

            for _ in range(steps):
                bus.poll()
                sim.step()
                time.sleep(step_period_s)

            end_units = read_present_position(client_fd, bus, device_id)
        finally:
            os.close(client_fd)

        if abs(end_units - target_units) >= abs(start_units - target_units):
            raise AssertionError(
                f"servo {device_id} did not move toward target: "
                f"start={start_units}, target={target_units}, end={end_units}"
            )

        return VerificationResult(
            serial_path=serial.path,
            start_units=start_units,
            target_units=target_units,
            end_units=end_units,
        )


def ping(client_fd: int, bus: St3215SerialBus, device_id: int) -> None:
    os.write(client_fd, st3215.encode_packet(device_id, st3215.PING))
    bus.poll()
    response = read_response(client_fd)
    expected = st3215.encode_status(device_id)
    if response != expected:
        raise AssertionError(f"bad ping response: {response.hex()} expected {expected.hex()}")


def write_goal_position(
    client_fd: int,
    bus: St3215SerialBus,
    device_id: int,
    target_units: int,
) -> None:
    payload = bytes([st3215.ADDR_GOAL_POSITION]) + st3215.write_u16_le(target_units) + b"\x00\x00\x00\x00"
    os.write(client_fd, st3215.encode_packet(device_id, st3215.WRITE_DATA, payload))
    bus.poll()
    response = read_response(client_fd)
    expected = st3215.encode_status(device_id)
    if response != expected:
        raise AssertionError(f"bad write response: {response.hex()} expected {expected.hex()}")


def read_present_position(client_fd: int, bus: St3215SerialBus, device_id: int) -> int:
    os.write(
        client_fd,
        st3215.encode_packet(
            device_id,
            st3215.READ_DATA,
            bytes([st3215.ADDR_PRESENT_POSITION, 2]),
        ),
    )
    bus.poll()
    response = read_response(client_fd)
    packet = bytearray(response)
    decoded = st3215.try_decode_packet(packet)
    if decoded is None:
        raise AssertionError(f"incomplete read response: {response.hex()}")
    if decoded.device_id != device_id or decoded.instruction != 0 or len(decoded.parameters) != 2:
        raise AssertionError(f"bad read response: {response.hex()}")
    return st3215.read_u16_le(decoded.parameters)


def read_response(client_fd: int, timeout_s: float = 0.2) -> bytes:
    deadline = time.monotonic() + timeout_s
    response = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = os.read(client_fd, 64)
        except BlockingIOError:
            chunk = b""
        if chunk:
            response.extend(chunk)
            packet = bytearray(response)
            try:
                if st3215.try_decode_packet(packet) is not None:
                    return bytes(response)
            except st3215.PacketError as error:
                raise AssertionError(f"invalid response packet: {response.hex()}") from error
        time.sleep(0.001)
    raise TimeoutError("timed out waiting for serial response")


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify SO101 sim ST3215 serial control loop")
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--target-units", type=int, default=3072)
    parser.add_argument("--steps", type=int, default=240)
    args = parser.parse_args()

    result = verify_serial_control_and_feedback(
        device_id=args.device_id,
        target_units=args.target_units,
        steps=args.steps,
    )
    print(
        "serial verification ok: "
        f"path={result.serial_path} "
        f"start={result.start_units} "
        f"target={result.target_units} "
        f"end={result.end_units}"
    )


if __name__ == "__main__":
    main()
