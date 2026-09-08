# Device-owned codecs and physical-bus runtime

## Decisions

- `protocol/` is not a runtime layer. Each concrete device owns its wire codec.
- A device has exactly one direction: sensor or actuator. A physical motor is
  represented by a paired sensor and actuator with the same physical identity.
- A profile device may select any compatible transport kind. Compatibility is
  validated by the device factory, not encoded in daemon conditionals.
- Physical transports are deduplicated by `(transport kind, canonical path)`.
  Consequently one CAN interface, serial descriptor, or EtherCAT master creates
  one bus runtime regardless of how many logical devices share it.
- Each bus runtime owns one receive worker and one send worker. Workers exchange
  fixed-capacity byte frames with devices through bounded queues. Lifecycle and
  descriptors are fixed before workers start; runtime faults latch until restart.
- A sensor decodes a received byte frame into its typed DDS message and publishes
  exactly once for that receive event. It never publishes from a timer.
- An actuator receives its typed DDS command, validates it, encodes a byte frame,
  and enqueues it for its physical bus. CAN, USB-CAN, and serial devices are
  independent. EtherCAT alone stages a complete execution-group epoch and commit.
- Cyclone DDS/CXX and IgH EtherCAT are required build dependencies. There are no
  feature macros or Unix-shared-memory fallback paths.

## Runtime ownership

`RobotIoDaemon` performs four startup phases:

1. Load and validate the profile into sensor and actuator device specifications.
2. Canonicalize transport keys and deduplicate the physical bus set.
3. Ask `TransportFactory` to open one transport and construct one `BusRuntime`
   for every key, then attach compatible devices to it.
4. Construct DDS endpoints, preallocate queues, and start bus receive/send workers.

Shutdown reverses phase four and then closes each physical transport exactly once.

## Byte-stream boundary

The transport/device boundary uses a transport-neutral bounded frame:

```cpp
struct DeviceFrame {
  std::uint32_t address;
  std::uint8_t size;
  std::array<std::byte, 256> bytes;
};
```

CAN transports use `address` as CAN ID and at most eight payload bytes. Serial
devices use it as a logical channel/device ID. EtherCAT uses a dedicated process
image view because it is cyclic shared data rather than a framed stream.

## OpenArm joint simulation

The OpenArm simulator binds a Linux SocketCAN interface (normally `vcan0`). The
native daemon opens that interface through the normal SocketCAN transport. Motors
1 through 7 map to OpenArm joints 1 through 7. The simulator accepts Damiao
enable (`0xFC`), disable (`0xFD`), zero (`0xFE`), and MIT frames, applies the MIT
position/velocity gains and feed-forward torque to MuJoCo, then emits canonical
Damiao feedback frames on the same interface.

The intended integration path is:

```text
policy-runtime-host -> DDS actuator topic -> actuator device codec
  -> bus TX queue -> vcan -> OpenArm simulator -> vcan -> bus RX queue
  -> sensor device codec -> DDS sensor topic -> policy-runtime-host
```

Tests cover codec round trips, transport-key deduplication, one runtime per bus,
enable/disable/MIT behavior, event-driven sensor publication, and a headless vcan
end-to-end run.
