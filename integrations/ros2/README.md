# ROS2 Integration Placeholder

This directory is intentionally non-core.

ROS2 is a future integration consumer of the robot I/O and policy RPC layers. The embodied policy runtime itself does not depend on ROS2, `ros2_control`, hardware drivers, or realtime servo loops.

Future ROS2 examples in this directory should:

- model ROS topics/services that enter the robot as `Sensor` implementations
- model ROS commands published by the robot as `Actuator` implementations
- treat the runtime as an external service
- avoid leaking ROS2 assumptions back into the core runtime
