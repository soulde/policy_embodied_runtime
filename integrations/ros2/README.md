# ROS2 Integration Placeholder

This directory is intentionally non-core.

ROS2 is a future integration consumer of the Python SDK. The embodied policy runtime itself does not depend on ROS2, `ros2_control`, hardware drivers, or realtime servo loops.

Future ROS2 examples in this directory should:

- use `policy_embodied_runtime.sdk.policy_client.PolicyClient`
- treat the runtime as an external service
- avoid leaking ROS2 assumptions back into the core server or SDK
