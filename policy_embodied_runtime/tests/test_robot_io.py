from policy_embodied_runtime.robot import RecordingActuator, RobotAction, RobotData, RpcActionActuator, RpcObservationSensor, StaticSensor
from policy_embodied_runtime.transport import TransportFrame
from policy_embodied_runtime.transport.serial import SerialConfig, SerialTransport


def test_static_sensor_and_recording_actuator_use_robot_data() -> None:
    data = RobotData()
    sensor = StaticSensor("remote", "buttons", {"a": True})
    actuator = RecordingActuator("telemetry")

    sensor.read(data)
    data.set_action(RobotAction({"motor": {"target": 1.0}}))
    actuator.write(data)

    assert data.sensors.get("buttons") == {"a": True}
    assert actuator.writes == [{"motor": {"target": 1.0}}]


def test_rpc_is_modeled_as_sensor_and_actuator() -> None:
    data = RobotData()
    sensor = RpcObservationSensor({"joint_position": {"values": [0.0]}})
    actuator = RpcActionActuator()

    sensor.read(data)
    data.set_action(RobotAction({"joint_position_delta": {"values": [0.1]}}))
    actuator.write(data)

    assert data.sensors.get("policy_rpc_observation") == {"joint_position": {"values": [0.0]}}
    assert actuator.response == {"joint_position_delta": {"values": [0.1]}}


def test_serial_config_from_profile_args() -> None:
    config = SerialConfig.from_args(
        {
            "path": "/tmp/tty-test",
            "baud_rate": "1000000",
            "read_buffer_len": "64",
            "timeout_s": "0.01",
        }
    )

    assert config.path == "/tmp/tty-test"
    assert config.baud_rate == 1000000
    assert config.read_buffer_len == 64
    assert config.timeout_s == 0.01


def test_serial_transport_uses_pyserial(monkeypatch) -> None:
    class FakeSerial:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.is_open = True
            self.in_waiting = 3
            self.written = bytearray()

        def write(self, payload):
            self.written.extend(payload)

        def flush(self):
            pass

        def read(self, read_len):
            assert read_len == 3
            return b"xyz"

        def close(self):
            self.is_open = False

    created = {}

    def fake_serial(**kwargs):
        created["port"] = FakeSerial(**kwargs)
        return created["port"]

    monkeypatch.setattr("policy_embodied_runtime.transport.serial.serial.Serial", fake_serial)
    transport = SerialTransport(SerialConfig(path="/dev/ttyUSB0", baud_rate=1000000))

    transport.open()
    transport.send(TransportFrame(id=0, payload=b"abc"))
    frame = transport.receive()
    transport.close()

    assert created["port"].kwargs["port"] == "/dev/ttyUSB0"
    assert created["port"].kwargs["baudrate"] == 1000000
    assert created["port"].written == b"abc"
    assert frame == TransportFrame(id=0, payload=b"xyz")
    assert transport.is_open is False
