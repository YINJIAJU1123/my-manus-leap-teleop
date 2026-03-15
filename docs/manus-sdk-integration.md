# MANUS SDK integration notes

This repository intentionally does not vendor the following components:

- `MANUS_Core_2.4.0_SDK`
- `SDKClient_Linux.out`
- `libManusSDK.so` and other MANUS binaries
- `manus_ros2` and `manus_ros2_msgs` packages copied from the vendor SDK

Why:

- Those files are vendor-controlled and likely covered by MANUS licensing terms.
- The Linux SDK client is an external runtime dependency, not part of your teleoperation algorithm.
- Public GitHub repos should not include third-party binaries unless redistribution is explicitly allowed.

Recommended structure:

- Public repo: this repository, containing your own ROS2 teleop code.
- Local/private dependency: your MANUS SDK installation and any MANUS-provided ROS2 bridge.

If you still want a fully self-contained version for yourself, keep a separate private repo that adds MANUS vendor files on top of this public repo.

Local startup sequence:

1. Terminal 1: run `scripts/run_manus_sdk_client.sh`, then choose menu option `1`.
2. Terminal 2: run `scripts/build_workspace.sh` once, then `scripts/run_glove_bridge.sh`.
3. Terminal 3: run `scripts/run_leap_teleop.sh`.

Optional MANUS ROS2 bridge mode:

- `launch/leap_manus_teleop.py` and `launch/leap_teleop_mode.py` are kept as examples.
- They require an external installation of `manus_ros2` and `manus_ros2_msgs` that is not shipped here.
