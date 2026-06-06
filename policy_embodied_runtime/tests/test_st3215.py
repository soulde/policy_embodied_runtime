import math
import threading
import time

from policy_embodied_runtime.robot import RobotData, ServoCommand, St3215ServoActuator, St3215ServoConfig, St3215ServoSensor
from policy_embodied_runtime.sim.soarm101 import st3215 as sim_st3215
from policy_embodied_runtime.sim.soarm101.virtual_serial import VirtualSerialEndpoint
from policy_embodied_runtime.transport.serial import SerialConfig, SerialTransport


class MiniSt3215Bus:
    def __init__(self, serial: VirtualSerialEndpoint) -> None:
        self.serial = serial
        self.buffer = bytearray()
        self.positions = {1: 2048}

    def poll(self) -> None:
        incoming = self.serial.read_available()
        if incoming:
            self.buffer.extend(incoming)
        while True:
            packet = sim_st3215.try_decode_packet(self.buffer)
            if packet is None:
                return
            self._handle_packet(packet)

    def _handle_packet(self, packet: sim_st3215.Packet) -> None:
        if packet.device_id not in self.positions:
            return
        if packet.instruction == sim_st3215.READ_DATA:
            address, length = packet.parameters
            if address == sim_st3215.ADDR_PRESENT_POSITION and length == 2:
                self.serial.write(
                    sim_st3215.encode_status(
                        packet.device_id,
                        parameters=sim_st3215.write_u16_le(self.positions[packet.device_id]),
                    )
                )
        elif packet.instruction == sim_st3215.WRITE_DATA:
            address = packet.parameters[0]
            if address == sim_st3215.ADDR_GOAL_POSITION:
                self.positions[packet.device_id] = sim_st3215.read_u16_le(packet.parameters[1:3])
            self.serial.write(sim_st3215.encode_status(packet.device_id))


class PollingThread:
    def __init__(self, bus: MiniSt3215Bus) -> None:
        self.bus = bus
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def __enter__(self) -> "PollingThread":
        self.thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop.set()
        self.thread.join(timeout=1)

    def _run(self) -> None:
        while not self.stop.is_set():
            self.bus.poll()
            time.sleep(0.001)


def test_st3215_actuator_and_sensor_use_virtual_serial() -> None:
    with VirtualSerialEndpoint.open() as serial_endpoint:
        bus = MiniSt3215Bus(serial_endpoint)
        transport = SerialTransport(
            SerialConfig(
                path=serial_endpoint.path,
                baud_rate=1000000,
                timeout_s=0.05,
                read_buffer_len=64,
            )
        )
        transport.open()
        try:
            config = St3215ServoConfig(
                servo_id=3,
                device_id=1,
                feedback_field="servo3_position",
                command_field="servo3_target",
                max_position_units=4095,
                speed_units=1000,
            )
            actuator = St3215ServoActuator("servo3", config, transport)
            sensor = St3215ServoSensor("servo3_feedback", config, transport)
            data = RobotData()
            data.commands.update(
                "servo3_target",
                ServoCommand(
                    servo_id=3,
                    target_position_rad=math.tau / 2.0,
                    enabled=True,
                ),
            )

            with PollingThread(bus):
                actuator.write(data)
                time.sleep(0.02)
                assert bus.positions[1] == 2048

                sensor.read(data)

            feedback = data.sensors.get("servo3_position")
            assert feedback.servo_id == 3
            assert feedback.raw_position == 2048
        finally:
            transport.close()
