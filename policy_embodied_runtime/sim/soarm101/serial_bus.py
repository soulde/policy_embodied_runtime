from __future__ import annotations

from . import st3215
from .sim import SoArm101Sim
from .virtual_serial import VirtualSerialEndpoint


class St3215SerialBus:
    def __init__(self, serial: VirtualSerialEndpoint, sim: SoArm101Sim) -> None:
        self.serial = serial
        self.sim = sim
        self.buffer = bytearray()

    def poll(self) -> None:
        incoming = self.serial.read_available()
        if incoming:
            self.buffer.extend(incoming)

        while True:
            try:
                packet = st3215.try_decode_packet(self.buffer)
            except st3215.PacketError:
                self.buffer.clear()
                return
            if packet is None:
                return
            self._handle_packet(packet)

    def _handle_packet(self, packet: st3215.Packet) -> None:
        if packet.device_id == st3215.BROADCAST_ID:
            self._handle_broadcast(packet)
            return

        if packet.device_id not in self.sim.servos:
            return

        if packet.instruction == st3215.PING:
            self.serial.write(st3215.encode_status(packet.device_id))
        elif packet.instruction == st3215.READ_DATA:
            self._handle_read(packet)
        elif packet.instruction == st3215.WRITE_DATA:
            self._handle_write(packet, respond=True)

    def _handle_broadcast(self, packet: st3215.Packet) -> None:
        if packet.instruction == st3215.WRITE_DATA:
            self._handle_write(packet, respond=False)

    def _handle_read(self, packet: st3215.Packet) -> None:
        if len(packet.parameters) != 2:
            self.serial.write(st3215.encode_status(packet.device_id, error=1))
            return

        address = packet.parameters[0]
        length = packet.parameters[1]
        if address == st3215.ADDR_PRESENT_POSITION and length == 2:
            position = self.sim.get_position_units(packet.device_id)
            self.serial.write(
                st3215.encode_status(packet.device_id, parameters=st3215.write_u16_le(position))
            )
        else:
            self.serial.write(st3215.encode_status(packet.device_id, error=1))

    def _handle_write(self, packet: st3215.Packet, respond: bool) -> None:
        if len(packet.parameters) < 3:
            if respond:
                self.serial.write(st3215.encode_status(packet.device_id, error=1))
            return

        address = packet.parameters[0]
        if address == st3215.ADDR_GOAL_POSITION:
            position = st3215.read_u16_le(packet.parameters[1:3])
            if packet.device_id == st3215.BROADCAST_ID:
                for device_id in self.sim.servos:
                    self.sim.set_goal_units(device_id, position)
            else:
                self.sim.set_goal_units(packet.device_id, position)

        if respond:
            self.serial.write(st3215.encode_status(packet.device_id))
