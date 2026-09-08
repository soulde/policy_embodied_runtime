# Generic Damiao CAN Motor Design

## Scope

Add a generic `damiao` private motor protocol without embedding a product
model name. The protocol must run over SocketCAN and over a USB-CAN adapter
exposed as a virtual serial port. Both transports share command/feedback
semantics; only frame wrapping differs.

## Runtime contract

Configuration is validated before entering the realtime loop. Realtime code
uses preallocated buffers and nonblocking I/O, performs one send attempt and a
bounded receive pass per cycle, and never retries, waits for recovery, reopens
devices, or reinitializes protocols. Any write error, short write, malformed
frame, range violation, or stale feedback latches a fault and publishes safe
output. Recovery is performed by process restart.

## Layering

`DamiaoSensor` and `DamiaoActuator` independently expose the directional
Sensor/Actuator interfaces. They delegate physical command/feedback conversion
to the Damiao codec. `SocketCanTransport` maps raw frames to CAN
ID/DLC/data, while `VirtualSerialTransport` maps them to the adapter's framed
serial stream. Transport code contains no motor semantics, and a single
physical transport may carry multiple protocols.

## Verification

Unit tests cover packing/unpacking, signed and unsigned bounds, invalid IDs and
lengths, and fault propagation. Integration tests use a fake transport and a
PTY to prove one-shot realtime behavior without requiring hardware.
