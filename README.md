# my-manus-leap-teleop

Public-ready ROS2 workspace for MANUS-to-LEAP teleoperation.

This repo keeps your own teleoperation code and LEAP assets, but does not vendor MANUS SDK binaries or MANUS vendor ROS2 packages.

## What is included

- `src/glove`: ZMQ-to-ROS2 glove bridge
- `src/telekinesis`: LEAP teleoperation nodes and URDF/mesh assets
- `launch/`: launch files for your current local workflow
- `scripts/`: helper scripts matching your terminal commands

## What is not included

- `MANUS_Core_2.4.0_SDK`
- `SDKClient_Linux.out`
- `libManusSDK.so`
- vendor packages such as `manus_ros2` and `manus_ros2_msgs`
- private CAD export config with Onshape keys

The reason is simple: those parts are vendor-controlled or secret-bearing and are not good candidates for a public GitHub repository.

## Recommended repo strategy

- Public GitHub repo: this repository
- Local machine: keep MANUS SDK installed separately
- Optional private repo: if you want a fully self-contained internal copy with vendor files

## Dependencies

System:

- Ubuntu with ROS2 Humble
- `colcon`
- `python3-pip`

Python:

```bash
pip install -r requirements.txt
```

## Build

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/build_workspace.sh
```

## Your local runtime flow

### Terminal 1

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_manus_sdk_client.sh
```

When the MANUS client menu appears, choose `1`.

### Terminal 2

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_glove_bridge.sh
```

Optional environment variables for glove routing:

```bash
export GLOVE_ZMQ_ENDPOINT=tcp://localhost:8000
export GLOVE_LEFT_SN=<left_glove_serial>
export GLOVE_RIGHT_SN=<right_glove_serial>
```

If serials are not provided, the bridge auto-assigns incoming devices.

### Terminal 3

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_leap_teleop.sh
```

This matches your current `leap_v1_ik.py` workflow.

## Optional MANUS ROS2 mode

The launch files below are retained for your own use:

- `launch/leap_manus_teleop.py`
- `launch/leap_teleop_mode.py`

They require an external installation of `manus_ros2` and `manus_ros2_msgs`.

`telekinesis/leap_ik.py` now supports glove-only fallback mode even when those MANUS message packages are absent.

## Notes

- The repo is intentionally structured as a standalone ROS2 workspace.
- `build/`, `install/`, and `log/` are gitignored.
- Package metadata is marked `Proprietary` for now. If you want this repo to be open source, pick a real license before publishing.

More detail: see `docs/manus-sdk-integration.md`.
