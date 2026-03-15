# my-manus-leap-teleop

ROS2 workspace for MANUS-to-LEAP teleoperation.

This repository contains the teleoperation logic, LEAP assets, launch files, and helper scripts used in the local workflow. It intentionally does not vendor MANUS SDK binaries or MANUS-provided ROS2 packages.

## 中文简介

这是我用于 `MANUS glove -> LEAP hand` 遥操作的 ROS2 工作空间。

仓库中保留了：

- `glove`：把本地 ZMQ 手套数据转成 ROS2 topic
- `telekinesis`：LEAP 手部控制、IK、URDF 和 mesh
- `launch`：本地启动用的 launch 文件
- `scripts`：和我实际运行流程一致的脚本

仓库中没有包含：

- `MANUS_Core_2.4.0_SDK`
- `SDKClient_Linux.out`
- `libManusSDK.so`
- `manus_ros2` / `manus_ros2_msgs`
- 含私钥的 CAD 导出配置

这样做的原因是这些内容属于厂商 SDK 或私密配置，不适合直接放进公开 GitHub 仓库。

## English Overview

This workspace contains:

- `src/glove`: ZMQ-to-ROS2 glove bridge
- `src/telekinesis`: LEAP teleoperation nodes and robot assets
- `launch/`: launch files for the local workflow
- `scripts/`: helper scripts matching the local runtime steps

This workspace does not include:

- MANUS SDK binaries
- MANUS vendor ROS2 bridge packages
- secret-bearing CAD export config

## Repository Layout

- `src/glove`
- `src/telekinesis`
- `launch`
- `scripts`
- `docs`

## Dependencies

System:

- Ubuntu
- ROS2 Humble
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

## Runtime Flow

### Terminal 1: start MANUS SDK client

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_manus_sdk_client.sh
```

When the MANUS client menu appears, choose `1`.

### Terminal 2: start glove bridge

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_glove_bridge.sh
```

Optional environment variables:

```bash
export GLOVE_ZMQ_ENDPOINT=tcp://localhost:8000
export GLOVE_LEFT_SN=<left_glove_serial>
export GLOVE_RIGHT_SN=<right_glove_serial>
```

If glove serials are not provided, the bridge auto-assigns incoming devices.

### Terminal 3: start LEAP teleop

```bash
cd /data/Yin_ros2_ws/my-manus-leap-teleop
./scripts/run_leap_teleop.sh
```

This matches the current `launch/leap_v1_ik.py` workflow.

## Optional MANUS ROS2 Mode

The following launch files are kept for local use:

- `launch/leap_manus_teleop.py`
- `launch/leap_teleop_mode.py`

They require an external installation of `manus_ros2` and `manus_ros2_msgs`.

`src/telekinesis/telekinesis/leap_ik.py` now supports glove-only fallback mode even when those MANUS message packages are not installed.

## Publication Notes

- The repo is structured as a standalone ROS2 workspace.
- `build/`, `install/`, and `log/` are gitignored.
- Package metadata is currently marked `Proprietary`.
- If this should become a true open-source project, choose and add a real license before wider publication.

More detail:

- `docs/manus-sdk-integration.md`
- `docs/publish-checklist.md`
