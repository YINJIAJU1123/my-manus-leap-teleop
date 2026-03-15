#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
from collections import deque
import json
import os
import queue
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
RIGHT_MODEL_PATH = BASE_DIR / "revo3_right_urdf_from_urdf.xml"
LEFT_MODEL_PATH = BASE_DIR / "revo3_left_urdf_from_urdf.xml"

FINGERTIP_SITES = {
    "拇指": "right_thumb_fingertip",
    "食指": "right_index_fingertip",
    "中指": "right_middle_fingertip",
    "无名指": "right_ring_fingertip",
    "小指": "right_little_fingertip",
}
FINGER_JOINTS = {
    "拇指": ["right_thumb_CMP_joint", "right_thumb_CMR_joint", "right_thumb_MCP_joint", "right_thumb_PIP_joint", "right_thumb_DIP_joint"],
    "食指": ["right_index_MPR_joint", "right_index_MCP_joint", "right_index_PIP_joint", "right_index_DIP_joint"],
    "中指": ["right_middle_MPR_joint", "right_middle_MCP_joint", "right_middle_PIP_joint", "right_middle_DIP_joint"],
    "无名指": ["right_ring_MPR_joint", "right_ring_MCP_joint", "right_ring_PIP_joint", "right_ring_DIP_joint"],
    "小指": ["right_little_MPR_joint", "right_little_MCP_joint", "right_little_PIP_joint", "right_little_DIP_joint"],
}

MANUS_FINGERTIP_NODE_IDS: dict[str, int] = {
    "拇指": 4,
    "食指": 9,
    "中指": 14,
    "无名指": 19,
    "小指": 24,
}
THUMB_DIP_NODE_ID = 3
THUMB_PIP_NODE_ID = 2
ERGONOMICS_TO_ROBOT_JOINT: dict[str, str] = {
    "IndexSpread": "right_index_MPR_joint",
    "IndexMCPStretch": "right_index_MCP_joint",
    "IndexPIPStretch": "right_index_PIP_joint",
    "IndexDIPStretch": "right_index_DIP_joint",
    "MiddleSpread": "right_middle_MPR_joint",
    "MiddleMCPStretch": "right_middle_MCP_joint",
    "MiddlePIPStretch": "right_middle_PIP_joint",
    "MiddleDIPStretch": "right_middle_DIP_joint",
    "RingSpread": "right_ring_MPR_joint",
    "RingMCPStretch": "right_ring_MCP_joint",
    "RingPIPStretch": "right_ring_PIP_joint",
    "RingDIPStretch": "right_ring_DIP_joint",
    "PinkySpread": "right_little_MPR_joint",
    "PinkyMCPStretch": "right_little_MCP_joint",
    "PinkyPIPStretch": "right_little_PIP_joint",
    "PinkyDIPStretch": "right_little_DIP_joint",
}

V3_MOTOR_COUNT = 23
JOINT_TO_MOTOR: dict[str, int] = {
    "right_thumb_CMP_joint": 19,
    "right_thumb_CMR_joint": 20,
    "right_thumb_MCP_joint": 16,
    "right_thumb_PIP_joint": 17,
    "right_thumb_DIP_joint": 18,
    "right_index_MPR_joint": 12,
    "right_index_MCP_joint": 13,
    "right_index_PIP_joint": 14,
    "right_index_DIP_joint": 15,
    "right_middle_MPR_joint": 8,
    "right_middle_MCP_joint": 9,
    "right_middle_PIP_joint": 10,
    "right_middle_DIP_joint": 11,
    "right_ring_MPR_joint": 4,
    "right_ring_MCP_joint": 5,
    "right_ring_PIP_joint": 6,
    "right_ring_DIP_joint": 7,
    "right_little_MPR_joint": 0,
    "right_little_MCP_joint": 1,
    "right_little_PIP_joint": 2,
    "right_little_DIP_joint": 3,
}
JOINT_NAMES_ORDER = list(JOINT_TO_MOTOR.keys())
MOTOR_MAX_DEG: dict[int, float] = {**{i: 90.0 for i in range(23)}, 19: 105.0, 20: 105.0}


def _import_sdk():
    sdk_python_candidates = [
        os.environ.get("STARK_SDK_PYTHON_DIR"),
        str(BASE_DIR / "python"),
        "/home/lixin/stark-serialport-example-revo3/python",
    ]
    for p in sdk_python_candidates:
        if p and Path(p).exists() and p not in sys.path:
            sys.path.insert(0, p)

    try:
        from common_imports import sdk as sdk_mod  # type: ignore

        if sdk_mod is not None:
            return sdk_mod
    except Exception:
        pass

    try:
        from bc_stark_sdk import main_mod as sdk_mod  # type: ignore

        return sdk_mod
    except Exception as e:
        raise ModuleNotFoundError(
            "Cannot import SDK. Set STARK_SDK_PYTHON_DIR to a folder containing common_imports.py, "
            "or install bc_stark_sdk."
        ) from e


def get_joint_limits(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray]:
    jlow = np.full(model.nq, -np.pi)
    jhigh = np.full(model.nq, np.pi)
    for j in range(model.njnt):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        adr = model.jnt_qposadr[j]
        lo, hi = float(model.jnt_range[j, 0]), float(model.jnt_range[j, 1])
        if np.isfinite(lo) and np.isfinite(hi) and lo < hi:
            jlow[adr] = lo
            jhigh[adr] = hi
    return jlow, jhigh


def get_rest_pose(jlow: np.ndarray, jhigh: np.ndarray) -> np.ndarray:
    q = np.zeros_like(jlow)
    for i in range(len(q)):
        q[i] = 0.0 if (jlow[i] <= 0.0 <= jhigh[i]) else jlow[i]
    return q


def _build_finger_qpos_adrs(model: mujoco.MjModel) -> dict[str, list[int]]:
    result: dict[str, list[int]] = {}
    for fname, jnames in FINGER_JOINTS.items():
        adrs = []
        for jname in jnames:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                adrs.append(int(model.jnt_qposadr[jid]))
        result[fname] = adrs
    return result


def _build_finger_dof_indices(model: mujoco.MjModel) -> dict[str, list[int]]:
    result: dict[str, list[int]] = {}
    for fname, jnames in FINGER_JOINTS.items():
        dofs = []
        for jname in jnames:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid >= 0:
                dofs.append(int(model.jnt_dofadr[jid]))
        result[fname] = dofs
    return result


def build_qpos_to_motor_converter(model: mujoco.MjModel):
    joint_info: list[tuple[int, float, float, int, float] | None] = []
    for name in JOINT_NAMES_ORDER:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            joint_info.append(None)
            continue
        adr = model.jnt_qposadr[jid]
        lo, hi = float(model.jnt_range[jid, 0]), float(model.jnt_range[jid, 1])
        motor_id = JOINT_TO_MOTOR[name]
        span = hi - lo
        if span <= 0:
            joint_info.append(None)
            continue
        span_deg = np.degrees(span)
        effective_max = min(MOTOR_MAX_DEG[motor_id], float(span_deg))
        joint_info.append((adr, lo, span, motor_id, effective_max))

    def convert(qpos: np.ndarray, wrist_default: float = 0.0) -> list[float]:
        out = [wrist_default] * V3_MOTOR_COUNT
        for info in joint_info:
            if info is None:
                continue
            adr, lo, span, motor_id, effective_max = info
            if adr >= len(qpos):
                continue
            ratio = float(np.clip((float(qpos[adr]) - lo) / span, 0.0, 1.0))
            out[motor_id] = float(np.clip(ratio * effective_max, 0.0, effective_max))
        return out

    return convert


def _discover_manus_setup_script() -> Path | None:
    env_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    for prefix in env_prefixes:
        if not prefix:
            continue
        prefix_path = Path(prefix)
        if (prefix_path / "share" / "manus_ros2_msgs").exists():
            setup_path = prefix_path.parent / "setup.bash"
            if setup_path.exists():
                return setup_path

    candidates = [Path.home() / "ros2_ws" / "install" / "setup.bash"]
    candidates.extend((Path.home() / "Manus").glob("*/ros2_ws/install/setup.bash"))
    for path in candidates:
        if path.exists():
            install_dir = path.parent
            if (install_dir / "manus_ros2_msgs").exists() or (install_dir / "share" / "manus_ros2_msgs").exists():
                return path
    return None


def _start_manus_bridge(shared_state: dict, manus_topic: str, node_name: str, debug: bool = False) -> subprocess.Popen | None:
    setup_path = _discover_manus_setup_script()
    if setup_path is None:
        if debug:
            print("[Teleop] 未找到 manus_ros2_msgs 对应的 setup.bash")
        return None

    helper_code = f"""
import json
import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

TOPIC = {manus_topic!r}
NODE_IDS = {sorted(set(list(MANUS_FINGERTIP_NODE_IDS.values()) + [THUMB_DIP_NODE_ID, THUMB_PIP_NODE_ID]))!r}
rclpy.init(args=None)
msg_type = get_message('manus_ros2_msgs/msg/ManusGlove')
node = Node({node_name!r})

def cb(msg):
    result = {{"tips": {{}}, "ergonomics": {{}}}}
    for raw_node in msg.raw_nodes:
        node_id = int(raw_node.node_id)
        if node_id not in NODE_IDS:
            continue
        pos = raw_node.pose.position
        result["tips"][str(node_id)] = [float(pos.x), float(pos.y), float(pos.z)]
    for ergo in msg.ergonomics:
        result["ergonomics"][str(ergo.type)] = float(ergo.value)
    sys.stdout.write(json.dumps(result) + '\\n')
    sys.stdout.flush()

node.create_subscription(msg_type, TOPIC, cb, 10)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
"""
    command = "\n".join([
        f"source {shlex.quote(str(setup_path))} >/dev/null 2>&1 || exit 1",
        "exec /usr/bin/python3 -u - <<'PY'",
        helper_code.strip(),
        "PY",
    ])
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )

    def _stdout_reader() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            tip_payload = payload.get("tips", payload)
            ergonomics_payload = payload.get("ergonomics", {})
            fingertip_points: dict[str, list[float]] = {}
            for finger_cn, node_id in MANUS_FINGERTIP_NODE_IDS.items():
                xyz = tip_payload.get(str(node_id))
                if isinstance(xyz, list) and len(xyz) == 3:
                    fingertip_points[finger_cn] = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
            shared_state["manus_tip_points"] = fingertip_points
            thumb_dip_xyz = tip_payload.get(str(THUMB_DIP_NODE_ID))
            if isinstance(thumb_dip_xyz, list) and len(thumb_dip_xyz) == 3:
                shared_state["manus_thumb_dip_point"] = [float(thumb_dip_xyz[0]), float(thumb_dip_xyz[1]), float(thumb_dip_xyz[2])]
            thumb_pip_xyz = tip_payload.get(str(THUMB_PIP_NODE_ID))
            if isinstance(thumb_pip_xyz, list) and len(thumb_pip_xyz) == 3:
                shared_state["manus_thumb_pip_point"] = [float(thumb_pip_xyz[0]), float(thumb_pip_xyz[1]), float(thumb_pip_xyz[2])]
            shared_state["manus_ergonomics"] = {str(k): float(v) for k, v in ergonomics_payload.items()}
            shared_state["manus_last_update"] = time.time()

    def _stderr_reader() -> None:
        assert proc.stderr is not None
        for line in proc.stderr:
            if debug and line.strip():
                print(f"[Teleop][{node_name}] {line.strip()}")

    threading.Thread(target=_stdout_reader, daemon=True).start()
    threading.Thread(target=_stderr_reader, daemon=True).start()
    return proc


def _transform_manus_xyz(
    xyz: list[float] | tuple[float, float, float] | np.ndarray,
    manus_xyz_scale: float,
    z_rotation_rad: float,
    out_x_sign: float,
    out_y_sign: float,
    out_z_sign: float,
) -> np.ndarray:
    x = float(xyz[0])
    y = float(xyz[1])
    z = float(xyz[2])
    rot_x = np.cos(z_rotation_rad) * x - np.sin(z_rotation_rad) * y
    rot_y = np.sin(z_rotation_rad) * x + np.cos(z_rotation_rad) * y
    return np.array(
        [
            out_x_sign * rot_x * manus_xyz_scale,
            out_y_sign * rot_y * manus_xyz_scale,
            out_z_sign * z * manus_xyz_scale,
        ],
        dtype=float,
    )


def _scale_thumb_target_xyz(thumb_xyz: np.ndarray, thumb_scale_xyz: np.ndarray) -> np.ndarray:
    return np.asarray(thumb_xyz, dtype=float) * np.asarray(thumb_scale_xyz, dtype=float)


def _build_transformed_manus_targets(
    manus_tip_points: dict[str, list[float]],
    manus_xyz_scale: float,
    z_rotation_rad: float,
    out_x_sign: float,
    out_y_sign: float,
    out_z_sign: float,
) -> dict[str, np.ndarray]:
    return {
        finger_cn: _transform_manus_xyz(xyz, manus_xyz_scale, z_rotation_rad, out_x_sign, out_y_sign, out_z_sign)
        for finger_cn, xyz in manus_tip_points.items()
        if finger_cn in MANUS_FINGERTIP_NODE_IDS
    }


def _apply_ema_to_manus_targets(
    previous_targets: dict[str, np.ndarray] | None,
    current_targets: dict[str, np.ndarray],
    ema_prev: float,
    ema_cur: float,
) -> dict[str, np.ndarray]:
    filtered_targets: dict[str, np.ndarray] = {}
    previous_targets = previous_targets or {}
    for finger_cn, current_xyz in current_targets.items():
        previous_xyz = previous_targets.get(finger_cn)
        if previous_xyz is None:
            filtered_targets[finger_cn] = np.asarray(current_xyz, dtype=float).copy()
        else:
            filtered_targets[finger_cn] = (
                ema_prev * np.asarray(previous_xyz, dtype=float) + ema_cur * np.asarray(current_xyz, dtype=float)
            )
    return filtered_targets


def _compute_xz_spread(points: dict[str, np.ndarray]) -> float:
    if not points:
        return 0.0
    arr = np.vstack(list(points.values()))
    x_span = float(arr[:, 0].max() - arr[:, 0].min())
    z_span = float(arr[:, 2].max() - arr[:, 2].min())
    return float(np.hypot(x_span, z_span))


def _compute_robot_reference_tip_points(
    mj_model: mujoco.MjModel,
    qpos: np.ndarray,
    site_ids: dict[str, int],
) -> dict[str, np.ndarray]:
    data = mujoco.MjData(mj_model)
    data.qpos[:] = np.asarray(qpos, dtype=float)
    mujoco.mj_forward(mj_model, data)
    return {
        finger_cn: data.site_xpos[site_id].copy()
        for finger_cn, site_id in site_ids.items()
        if site_id >= 0
    }


def _apply_ergonomics_to_q(
    q: np.ndarray,
    ergonomics: dict[str, float],
    joint_name_to_adr: dict[str, int],
    jlow: np.ndarray,
    jhigh: np.ndarray,
    mpr_bias_deg: float,
    spread_sign: float,
) -> np.ndarray:
    mapped_q = np.asarray(q, dtype=float).copy()
    for ergo_name, joint_name in ERGONOMICS_TO_ROBOT_JOINT.items():
        value_deg = ergonomics.get(ergo_name)
        adr = joint_name_to_adr.get(joint_name)
        if value_deg is None or adr is None:
            continue
        if ergo_name.endswith("Spread"):
            value_deg = spread_sign * float(value_deg) + mpr_bias_deg
        value_rad = np.deg2rad(float(value_deg))
        mapped_q[adr] = np.clip(value_rad, jlow[adr], jhigh[adr])
    return mapped_q


def _solve_thumb_ik(
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
    current_q: np.ndarray,
    thumb_target_xyz: np.ndarray,
    thumb_dip_target_xyz: np.ndarray | None,
    thumb_pip_target_xyz: np.ndarray | None,
    finger_qpos_adrs: dict[str, list[int]],
    finger_dof_indices: dict[str, list[int]],
    site_ids: dict[str, int],
    thumb_dip_body_id: int,
    thumb_pip_body_id: int,
    jlow: np.ndarray,
    jhigh: np.ndarray,
) -> np.ndarray:
    q = np.asarray(current_q, dtype=float).copy()
    adrs = finger_qpos_adrs["拇指"]
    dofs = finger_dof_indices["拇指"]
    thumb_site_id = site_ids["拇指"]
    jacp_tip = np.zeros((3, mj_model.nv))
    jacr_tip = np.zeros((3, mj_model.nv))
    jacp_dip = np.zeros((3, mj_model.nv))
    jacr_dip = np.zeros((3, mj_model.nv))
    jacp_pip = np.zeros((3, mj_model.nv))
    jacr_pip = np.zeros((3, mj_model.nv))

    for _ in range(40):
        mj_data.qpos[:] = q
        mujoco.mj_forward(mj_model, mj_data)

        tip_curr = mj_data.site_xpos[thumb_site_id].copy()
        tip_err = thumb_target_xyz - tip_curr
        err_blocks = [2.0 * tip_err]
        mujoco.mj_jac(mj_model, mj_data, jacp_tip, jacr_tip, tip_curr, mj_model.site_bodyid[thumb_site_id])
        jac_blocks = [2.0 * jacp_tip[:, dofs]]

        if thumb_dip_target_xyz is not None and thumb_dip_body_id >= 0:
            dip_curr = mj_data.xpos[thumb_dip_body_id].copy()
            dip_err = thumb_dip_target_xyz - dip_curr
            err_blocks.append(0.1 * dip_err)
            mujoco.mj_jac(mj_model, mj_data, jacp_dip, jacr_dip, dip_curr, thumb_dip_body_id)
            jac_blocks.append(0.1 * jacp_dip[:, dofs])

        if thumb_pip_target_xyz is not None and thumb_pip_body_id >= 0:
            pip_curr = mj_data.xpos[thumb_pip_body_id].copy()
            pip_err = thumb_pip_target_xyz - pip_curr
            err_blocks.append(0.1 * pip_err)
            mujoco.mj_jac(mj_model, mj_data, jacp_pip, jacr_pip, pip_curr, thumb_pip_body_id)
            jac_blocks.append(0.1 * jacp_pip[:, dofs])

        err = np.concatenate(err_blocks)
        if np.linalg.norm(err) < 5e-4:
            break

        J = np.vstack(jac_blocks)
        JJT = J @ J.T + (2e-2 * 2e-2) * np.eye(J.shape[0])
        dq = J.T @ np.linalg.solve(JJT, err)
        for i, adr in enumerate(adrs):
            q[adr] = np.clip(q[adr] + 0.30 * dq[i], jlow[adr], jhigh[adr])
    return q


def _push_latest_motor_target(send_queue: queue.Queue, motor_pos: list[float]) -> None:
    while not send_queue.empty():
        try:
            send_queue.get_nowait()
        except queue.Empty:
            break
    send_queue.put_nowait(list(motor_pos))


def _run_mujoco_viewer(model_path: Path, shared_state: dict) -> None:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            q = shared_state.get("q")
            if q is not None and len(q) == model.nq:
                data.qpos[:] = q
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1 / 60.0)


@dataclass
class HandProfile:
    side: str
    model_path: Path
    manus_topic: str
    enabled: bool
    x_sign: float
    y_sign: float
    z_sign: float
    spread_sign: float
    mpr_bias_deg: float = 15.0
    manus_z_rotation_rad: float = np.pi / 2.0
    manus_scale_xz: float = 1.13
    thumb_scale_xyz: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0], dtype=float))
    auto_calibrate_seconds: float = 0.0
    ema_prev: float = 0.9
    ema_cur: float = 0.1
    retarget_hz: float = 60.0
    hardware_hz: float = 200.0
    filter_window: int = 20
    port: str | None = None
    slave_id: int = 1


@dataclass
class QuinticSegment:
    """Quintic polynomial trajectory in normalized time s ∈ [0,1].

    p(s)   = coeffs @ [1, s, s², s³, s⁴, s⁵]
    v(t)   = (coeffs @ [0, 1, 2s, 3s², 4s³, 5s⁴]) / T
    a(t)   = (coeffs @ [0, 0, 2, 6s, 12s², 20s³]) / T²

    where s = clip((t - t_start) / T, 0, 1), T = t_end - t_start.
    """

    coeffs: np.ndarray  # shape (n_motors, 6)
    t_start: float
    t_end: float
    p_end: np.ndarray   # final position, used for clamping after segment ends


def _make_quintic_segment(
    p0: np.ndarray,
    v0: np.ndarray,
    a0: np.ndarray,
    p1: np.ndarray,
    v1: np.ndarray,
    a1: np.ndarray,
    t_start: float,
    duration: float,
) -> QuinticSegment:
    """Create a quintic Hermite segment matching position, velocity, and
    acceleration at both endpoints.  All kinematic quantities are in
    physical units (degrees, degrees/s, degrees/s²); duration is in seconds.
    """
    T = max(duration, 1e-6)
    c0 = p0.copy()
    c1 = v0 * T
    c2 = a0 * (T ** 2) / 2.0

    # Solve the 3×3 system for [c3, c4, c5] from the end-point constraints:
    #   c0+c1+c2+c3+c4+c5          = p1
    #   c1+2c2+3c3+4c4+5c5         = v1·T
    #   2c2+6c3+12c4+20c5          = a1·T²
    r0 = p1 - c0 - c1 - c2
    r1 = v1 * T - c1 - 2.0 * c2
    r2 = a1 * (T ** 2) - 2.0 * c2

    # Matrix rows stacked as (3, n_motors); solve A·X = R column-wise.
    A = np.array([[1.0, 1.0, 1.0],
                  [3.0, 4.0, 5.0],
                  [6.0, 12.0, 20.0]], dtype=float)
    R = np.stack([r0, r1, r2], axis=0)   # (3, n_motors)
    X = np.linalg.solve(A, R)            # (3, n_motors)

    coeffs = np.stack([c0, c1, c2, X[0], X[1], X[2]], axis=-1)  # (n_motors, 6)
    return QuinticSegment(coeffs=coeffs, t_start=t_start, t_end=t_start + T, p_end=p1.copy())


def _eval_quintic(seg: QuinticSegment, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (position, velocity, acceleration) at time *t* from a QuinticSegment."""
    T = seg.t_end - seg.t_start
    if T <= 0.0:
        return seg.p_end.copy(), np.zeros(seg.coeffs.shape[0]), np.zeros(seg.coeffs.shape[0])
    s = float(np.clip((t - seg.t_start) / T, 0.0, 1.0))
    c = seg.coeffs  # (n_motors, 6)
    pos = c @ np.array([1.0, s, s**2, s**3, s**4, s**5])
    vel = (c @ np.array([0.0, 1.0, 2*s, 3*s**2, 4*s**3, 5*s**4])) / T
    acc = (c @ np.array([0.0, 0.0, 2.0, 6*s, 12*s**2, 20*s**3])) / (T ** 2)
    return pos, vel, acc


@dataclass
class HandRuntime:
    profile: HandProfile
    mj_model: mujoco.MjModel
    mj_data: mujoco.MjData
    jlow: np.ndarray
    jhigh: np.ndarray
    current_q: np.ndarray
    qpos_to_motor: object
    finger_qpos_adrs: dict[str, list[int]]
    finger_dof_indices: dict[str, list[int]]
    site_ids: dict[str, int]
    thumb_dip_body_id: int
    thumb_pip_body_id: int
    joint_name_to_adr: dict[str, int]
    robot_reference_spread: float
    shared_state: dict
    send_queue: queue.Queue
    read_result: dict
    manus_proc: subprocess.Popen | None = None
    device = None
    conn_result: dict = field(default_factory=dict)
    calibrated: bool = False
    calibration_started_at: float = field(default_factory=time.time)
    calibration_samples: list[dict[str, np.ndarray]] = field(default_factory=list)
    manus_scale_xz: float = 1.13
    manus_filtered_targets: dict[str, np.ndarray] = field(default_factory=dict)
    last_applied_update: float | None = None
    quintic_segment: QuinticSegment | None = None
    interp_state: np.ndarray | None = None  # shape (3, V3_MOTOR_COUNT): [pos, vel, acc]
    segment_lock: threading.Lock = field(default_factory=threading.Lock)
    viz_cmd: list[float] | None = None  # latest commanded motor positions (deg), written by send loop


def build_runtime(profile: HandProfile) -> HandRuntime:
    model = mujoco.MjModel.from_xml_path(str(profile.model_path))
    data = mujoco.MjData(model)
    jlow, jhigh = get_joint_limits(model)
    current_q = get_rest_pose(jlow, jhigh).copy()
    qpos_to_motor = build_qpos_to_motor_converter(model)
    finger_qpos_adrs = _build_finger_qpos_adrs(model)
    finger_dof_indices = _build_finger_dof_indices(model)
    site_ids = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, s) for n, s in FINGERTIP_SITES.items()}
    thumb_dip_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_thumb_DIP_Link")
    thumb_pip_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_thumb_PIP_Link")
    joint_name_to_adr = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid): int(model.jnt_qposadr[jid])
        for jid in range(model.njnt)
        if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) is not None
    }
    robot_reference_spread = _compute_xz_spread(_compute_robot_reference_tip_points(model, current_q, site_ids))
    shared_state = {
        "q": current_q.copy(),
        "manus_tip_points": {},
        "manus_thumb_dip_point": None,
        "manus_thumb_pip_point": None,
        "manus_ergonomics": {},
        "manus_last_update": None,
    }
    runtime = HandRuntime(
        profile=profile,
        mj_model=model,
        mj_data=data,
        jlow=jlow,
        jhigh=jhigh,
        current_q=current_q,
        qpos_to_motor=qpos_to_motor,
        finger_qpos_adrs=finger_qpos_adrs,
        finger_dof_indices=finger_dof_indices,
        site_ids=site_ids,
        thumb_dip_body_id=thumb_dip_body_id,
        thumb_pip_body_id=thumb_pip_body_id,
        joint_name_to_adr=joint_name_to_adr,
        robot_reference_spread=robot_reference_spread,
        shared_state=shared_state,
        send_queue=queue.Queue(maxsize=1),
        read_result={"positions": None},
        calibrated=(profile.auto_calibrate_seconds <= 0),
        manus_scale_xz=float(profile.manus_scale_xz),
    )
    return runtime


def _run_hardware_send_loop(runtime: HandRuntime) -> None:
    """Thread that runs at hardware_hz, evaluates the quintic segment, applies
    a sliding-window average filter, and pushes the result to the send queue."""
    interval = 1.0 / runtime.profile.hardware_hz if runtime.profile.hardware_hz > 0 else 0.005
    win = max(1, runtime.profile.filter_window)
    filter_buf: deque[np.ndarray] = deque(maxlen=win)
    last_sent: np.ndarray | None = None
    dead_zone = 0.1  # degrees
    while True:
        t0 = time.time()
        with runtime.segment_lock:
            seg = runtime.quintic_segment
        if seg is not None:
            now = time.time()
            pos, vel, acc = _eval_quintic(seg, now)
            pos = np.clip(pos, 0.0, 105.0)
            with runtime.segment_lock:
                runtime.interp_state = np.stack([pos, vel, acc], axis=0)
            filter_buf.append(pos)
            filtered_pos = np.mean(filter_buf, axis=0)
            if last_sent is None:
                last_sent = filtered_pos.copy()
            else:
                mask = np.abs(filtered_pos - last_sent) >= dead_zone
                last_sent[mask] = filtered_pos[mask]
            runtime.viz_cmd = last_sent.tolist()
            _push_latest_motor_target(runtime.send_queue, last_sent.tolist())
        elapsed = time.time() - t0
        time.sleep(max(0.0, interval - elapsed))


def start_hardware_loop(runtime: HandRuntime, detected_device=None) -> None:
    profile = runtime.profile
    sdk = _import_sdk()

    async def _connect():
        if profile.port:
            ctx = await sdk.modbus_open(profile.port, sdk.Baudrate.Baud5Mbps)
            return ctx, profile.slave_id
        if detected_device is not None:
            ctx = await sdk.init_from_detected(detected_device)
            sid = getattr(detected_device, "slave_id", profile.slave_id)
            return ctx, sid
        devices = await sdk.auto_detect(scan_all=False, port=None, protocol=None)
        if not devices:
            raise RuntimeError("未检测到设备")
        d = devices[0]
        ctx = await sdk.init_from_detected(d)
        return ctx, d.slave_id

    async def _queue_processor(dev, sid, q_send: queue.Queue):
        while True:
            try:
                motor_pos = q_send.get(timeout=0.01)
            except queue.Empty:
                await asyncio.sleep(0.001)
                continue
            if len(motor_pos) == V3_MOTOR_COUNT:
                await dev.v3_set_all_motor_positions(sid, motor_pos)

    async def _read_motor_loop(dev, sid, result_dict: dict):
        while True:
            try:
                pos = await dev.v3_get_all_motor_positions(sid)
                result_dict["positions"] = list(pos) if pos else None
            except Exception:
                result_dict["positions"] = None
            await asyncio.sleep(0.05)

    def _run_loop():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        dev, sid = loop.run_until_complete(_connect())
        runtime.conn_result["device"] = dev
        runtime.conn_result["slave_id"] = sid
        runtime.device = dev
        loop.create_task(_queue_processor(dev, sid, runtime.send_queue))
        loop.create_task(_read_motor_loop(dev, sid, runtime.read_result))
        loop.run_forever()

    thread = threading.Thread(target=_run_loop, daemon=True)
    thread.start()
    for _ in range(50):
        if "device" in runtime.conn_result:
            print(f"[Teleop][{profile.side}] 已连接设备 slave_id={runtime.conn_result['slave_id']}")
            threading.Thread(target=_run_hardware_send_loop, args=(runtime,), daemon=True).start()
            return
        time.sleep(0.1)
    raise RuntimeError(f"[{profile.side}] 设备连接超时")


# Finger → joint names, used for subplot grouping in the visualizer.
_VIZ_FINGER_JOINTS: list[tuple[str, list[str]]] = [
    ("Thumb",  ["right_thumb_CMP_joint", "right_thumb_CMR_joint", "right_thumb_MCP_joint",
                "right_thumb_PIP_joint", "right_thumb_DIP_joint"]),
    ("Index",  ["right_index_MPR_joint",  "right_index_MCP_joint",
                "right_index_PIP_joint",  "right_index_DIP_joint"]),
    ("Middle", ["right_middle_MPR_joint", "right_middle_MCP_joint",
                "right_middle_PIP_joint", "right_middle_DIP_joint"]),
    ("Ring",   ["right_ring_MPR_joint",   "right_ring_MCP_joint",
                "right_ring_PIP_joint",   "right_ring_DIP_joint"]),
    ("Little", ["right_little_MPR_joint", "right_little_MCP_joint",
                "right_little_PIP_joint", "right_little_DIP_joint"]),
]
_VIZ_NCOLS = 5  # max joints per finger (thumb has 5)


def _run_retarget_loop(
    runtimes: list,
    physical_mode: bool,
    debug: bool,
    stop_event: threading.Event,
) -> None:
    """The 60 Hz retarget loop, extracted so it can run in a background thread
    when the matplotlib visualizer occupies the main thread."""
    try:
        while not stop_event.is_set():
            loop_start = time.time()
            next_rate = min(rt.profile.retarget_hz for rt in runtimes if rt.profile.retarget_hz > 0)
            for runtime in runtimes:
                p = runtime.profile
                manus_tip_points = runtime.shared_state.get("manus_tip_points") or {}
                manus_thumb_dip_point = runtime.shared_state.get("manus_thumb_dip_point")
                manus_thumb_pip_point = runtime.shared_state.get("manus_thumb_pip_point")
                manus_ergonomics = runtime.shared_state.get("manus_ergonomics") or {}
                manus_last_update = runtime.shared_state.get("manus_last_update")

                if not runtime.calibrated:
                    elapsed = time.time() - runtime.calibration_started_at
                    if manus_tip_points:
                        sample = _build_transformed_manus_targets(
                            manus_tip_points,
                            manus_xyz_scale=1.0,
                            z_rotation_rad=p.manus_z_rotation_rad,
                            out_x_sign=p.x_sign,
                            out_y_sign=p.y_sign,
                            out_z_sign=p.z_sign,
                        )
                        if sample:
                            runtime.calibration_samples.append(sample)
                    if elapsed >= p.auto_calibrate_seconds:
                        if runtime.calibration_samples:
                            averaged_points: dict[str, np.ndarray] = {}
                            for finger_cn in MANUS_FINGERTIP_NODE_IDS:
                                finger_samples = [s[finger_cn] for s in runtime.calibration_samples if finger_cn in s]
                                if finger_samples:
                                    averaged_points[finger_cn] = np.mean(np.vstack(finger_samples), axis=0)
                            manus_spread = _compute_xz_spread(averaged_points)
                            if manus_spread > 1e-6 and runtime.robot_reference_spread > 1e-6:
                                runtime.manus_scale_xz = float(np.clip(runtime.robot_reference_spread / manus_spread, 0.1, 10.0))
                                print(f"[Teleop][{p.side}] 标定完成 scale_xz={runtime.manus_scale_xz:.3f}")
                            else:
                                print(f"[Teleop][{p.side}] 标定失败，回退 scale_xz={runtime.manus_scale_xz:.3f}")
                        runtime.calibrated = True
                    continue

                if manus_tip_points and manus_last_update and manus_last_update != runtime.last_applied_update:
                    transformed_targets = _build_transformed_manus_targets(
                        manus_tip_points,
                        manus_xyz_scale=runtime.manus_scale_xz,
                        z_rotation_rad=p.manus_z_rotation_rad,
                        out_x_sign=p.x_sign,
                        out_y_sign=p.y_sign,
                        out_z_sign=p.z_sign,
                    )
                    thumb_target: dict[str, np.ndarray] = {}
                    if "拇指" in transformed_targets:
                        thumb_target["拇指"] = _scale_thumb_target_xyz(transformed_targets["拇指"], p.thumb_scale_xyz)
                    runtime.manus_filtered_targets = _apply_ema_to_manus_targets(
                        runtime.manus_filtered_targets, thumb_target, p.ema_prev, p.ema_cur
                    )
                    thumb_dip_target = None
                    if manus_thumb_dip_point is not None:
                        thumb_dip_target = _scale_thumb_target_xyz(
                            _transform_manus_xyz(
                                manus_thumb_dip_point,
                                runtime.manus_scale_xz,
                                p.manus_z_rotation_rad,
                                p.x_sign,
                                p.y_sign,
                                p.z_sign,
                            ),
                            p.thumb_scale_xyz,
                        )
                    thumb_pip_target = None
                    if manus_thumb_pip_point is not None:
                        thumb_pip_target = _scale_thumb_target_xyz(
                            _transform_manus_xyz(
                                manus_thumb_pip_point,
                                runtime.manus_scale_xz,
                                p.manus_z_rotation_rad,
                                p.x_sign,
                                p.y_sign,
                                p.z_sign,
                            ),
                            p.thumb_scale_xyz,
                        )
                    new_q = runtime.current_q.copy()
                    if "拇指" in runtime.manus_filtered_targets:
                        new_q = _solve_thumb_ik(
                            runtime.mj_model,
                            runtime.mj_data,
                            runtime.current_q,
                            runtime.manus_filtered_targets["拇指"],
                            thumb_dip_target,
                            thumb_pip_target,
                            runtime.finger_qpos_adrs,
                            runtime.finger_dof_indices,
                            runtime.site_ids,
                            runtime.thumb_dip_body_id,
                            runtime.thumb_pip_body_id,
                            runtime.jlow,
                            runtime.jhigh,
                        )
                    new_q = _apply_ergonomics_to_q(
                        new_q,
                        manus_ergonomics,
                        runtime.joint_name_to_adr,
                        runtime.jlow,
                        runtime.jhigh,
                        mpr_bias_deg=p.mpr_bias_deg,
                        spread_sign=p.spread_sign,
                    )
                    runtime.current_q = new_q
                    runtime.shared_state["q"] = new_q.copy()
                    if physical_mode:
                        new_motor = np.array(runtime.qpos_to_motor(np.asarray(new_q)), dtype=float)
                        duration = 1.0 / p.retarget_hz if p.retarget_hz > 0 else 1.0 / 60.0
                        now = time.time()
                        with runtime.segment_lock:
                            if runtime.interp_state is not None:
                                p0 = runtime.interp_state[0]
                                v0 = runtime.interp_state[1]
                                a0 = runtime.interp_state[2]
                            else:
                                p0 = new_motor.copy()
                                v0 = np.zeros(V3_MOTOR_COUNT, dtype=float)
                                a0 = np.zeros(V3_MOTOR_COUNT, dtype=float)
                            runtime.quintic_segment = _make_quintic_segment(
                                p0, v0, a0,
                                new_motor,
                                np.zeros(V3_MOTOR_COUNT, dtype=float),
                                np.zeros(V3_MOTOR_COUNT, dtype=float),
                                now, duration,
                            )
                    runtime.last_applied_update = manus_last_update
                    if debug:
                        print(f"[Teleop][{p.side}] update thumb={bool(runtime.manus_filtered_targets)} ergo={len(manus_ergonomics)}")

            sleep_time = max(0.0, (1.0 / next_rate) - (time.time() - loop_start)) if next_rate > 0 else 0.0
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass
    finally:
        for runtime in runtimes:
            if runtime.manus_proc is not None and runtime.manus_proc.poll() is None:
                runtime.manus_proc.terminate()


def _run_visualization(runtimes: list, history_seconds: float = 8.0) -> None:
    """Real-time matplotlib plot of the commanded motor positions for every joint.
    Must be called from the main thread.  Blocks until the window is closed."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[Viz] matplotlib not installed — skipping plot.")
        return

    display_hz = 30.0
    interval = 1.0 / display_hz
    n_history = int(history_seconds * display_hz)

    # Build per-finger motor-index lookup (joint name → motor id).
    finger_motor_ids: list[tuple[str, list[tuple[str, int, float]]]] = []
    for finger_label, jnames in _VIZ_FINGER_JOINTS:
        joints = []
        for jname in jnames:
            mid = JOINT_TO_MOTOR.get(jname)
            if mid is not None:
                short = jname.split("_")[-2]  # e.g. "CMP", "MCP", "PIP"…
                joints.append((short, mid, MOTOR_MAX_DEG.get(mid, 90.0)))
        finger_motor_ids.append((finger_label, joints))

    n_rows = len(finger_motor_ids)  # 5 fingers

    plt.ion()
    figs: list = []
    # all_lines[fig_idx][row][col] = Line2D
    all_lines: list[list[list]] = []
    all_hists: list[list[list[deque]]] = []

    for runtime in runtimes:
        fig, axs = plt.subplots(
            n_rows, _VIZ_NCOLS,
            figsize=(3 * _VIZ_NCOLS, 2.2 * n_rows),
            squeeze=False,
        )
        fig.suptitle(
            f"{runtime.profile.side.capitalize()} hand — commanded motor positions (deg)",
            fontsize=11,
        )

        rt_lines: list[list] = []
        rt_hists: list[list[deque]] = []

        for row, (finger_label, joints) in enumerate(finger_motor_ids):
            row_lines: list = []
            row_hists: list[deque] = []
            for col in range(_VIZ_NCOLS):
                ax = axs[row, col]
                if col < len(joints):
                    short, mid, maxd = joints[col]
                    ax.set_title(f"{finger_label}\n{short}", fontsize=7, pad=2)
                    ax.set_ylim(-2, maxd + 5)
                    ax.set_xlim(0, n_history)
                    ax.set_xticks([])
                    ax.set_yticks([0, int(maxd)])
                    ax.tick_params(labelsize=6)
                    ax.axhline(0, color="gray", lw=0.4, ls="--")
                    ax.axhline(maxd, color="gray", lw=0.4, ls="--")
                    line, = ax.plot([], [], lw=1.2, color="tab:blue")
                    row_lines.append(line)
                    row_hists.append(deque(maxlen=n_history))
                else:
                    ax.set_visible(False)
                    row_lines.append(None)
                    row_hists.append(deque(maxlen=n_history))
            rt_lines.append(row_lines)
            rt_hists.append(row_hists)

        fig.tight_layout()
        figs.append(fig)
        all_lines.append(rt_lines)
        all_hists.append(rt_hists)

    try:
        while all(plt.fignum_exists(f.number) for f in figs):
            for fi, (runtime, fig) in enumerate(zip(runtimes, figs)):
                cmd = runtime.viz_cmd
                for row, (_, joints) in enumerate(finger_motor_ids):
                    for col, (_, mid, _maxd) in enumerate(joints):
                        h = all_hists[fi][row][col]
                        if cmd is not None and mid < len(cmd):
                            h.append(cmd[mid])
                        ydata = list(h)
                        line = all_lines[fi][row][col]
                        if line is not None and ydata:
                            line.set_xdata(range(len(ydata)))
                            line.set_ydata(ydata)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
            plt.pause(interval)
    except KeyboardInterrupt:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Dual-hand Manus teleop for Revo3")
    parser.add_argument("--left", dest="left", action="store_true", help="启用左手（默认启用）")
    parser.add_argument("--no-left", dest="left", action="store_false", help="禁用左手")
    parser.add_argument("--right", dest="right", action="store_true", help="启用右手（默认启用）")
    parser.add_argument("--no-right", dest="right", action="store_false", help="禁用右手")
    parser.set_defaults(left=True, right=True)
    parser.add_argument("--left_manus_topic", default="/manus_glove_0")
    parser.add_argument("--right_manus_topic", default="/manus_glove_1")
    parser.add_argument("--sim-only", action="store_true", help="仅仿真，不连接硬件")
    parser.add_argument("--left-port", default=None)
    parser.add_argument("--right-port", default=None)
    parser.add_argument("--left-slave-id", type=int, default=1)
    parser.add_argument("--right-slave-id", type=int, default=1)
    parser.add_argument("--retarget-hz", type=float, default=60.0)
    parser.add_argument("--hardware-hz", type=float, default=200.0)
    parser.add_argument("--filter-window", type=int, default=5, help="sliding-average window size for hardware commands")
    parser.add_argument("--manus-scale-xz", type=float, default=1.13)
    parser.add_argument("--manus-auto-calibrate-seconds", type=float, default=0.0)
    parser.add_argument("--thumb-scale-x", type=float, default=1.0)
    parser.add_argument("--thumb-scale-y", type=float, default=1.0)
    parser.add_argument("--thumb-scale-z", type=float, default=1.0)
    parser.add_argument("--plot", action="store_true", help="show real-time joint command plot")
    parser.add_argument("--plot-history", type=float, default=8.0, metavar="SEC",
                        help="seconds of history shown in the plot (default 8)")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    if not args.left and not args.right:
        print("[Teleop] 左右手都被禁用，退出。")
        return 1
    if args.left and not LEFT_MODEL_PATH.exists():
        print(f"[Teleop] 左手模型不存在: {LEFT_MODEL_PATH}")
        return 1
    if args.right and not RIGHT_MODEL_PATH.exists():
        print(f"[Teleop] 右手模型不存在: {RIGHT_MODEL_PATH}")
        return 1

    thumb_scale_xyz = np.array([args.thumb_scale_x, args.thumb_scale_y, args.thumb_scale_z], dtype=float)
    profiles: list[HandProfile] = []
    if args.left:
        profiles.append(
            HandProfile(
                side="left",
                model_path=LEFT_MODEL_PATH,
                manus_topic=args.left_manus_topic,
                enabled=True,
                x_sign=1.0,
                y_sign=-1.0,
                z_sign=1.0,
                spread_sign=-1.0,
                mpr_bias_deg=15.0,
                manus_scale_xz=args.manus_scale_xz,
                auto_calibrate_seconds=args.manus_auto_calibrate_seconds,
                thumb_scale_xyz=thumb_scale_xyz.copy(),
                retarget_hz=args.retarget_hz,
                hardware_hz=args.hardware_hz,
                filter_window=args.filter_window,
                port=args.left_port,
                slave_id=args.left_slave_id,
            )
        )
    if args.right:
        profiles.append(
            HandProfile(
                side="right",
                model_path=RIGHT_MODEL_PATH,
                manus_topic=args.right_manus_topic,
                enabled=True,
                x_sign=1.0,
                y_sign=-1.0,
                z_sign=1.0,
                spread_sign=-1.0,
                manus_scale_xz=args.manus_scale_xz,
                auto_calibrate_seconds=args.manus_auto_calibrate_seconds,
                thumb_scale_xyz=thumb_scale_xyz.copy(),
                retarget_hz=args.retarget_hz,
                hardware_hz=args.hardware_hz,
                filter_window=args.filter_window,
                port=args.right_port,
                slave_id=args.right_slave_id,
            )
        )

    runtimes = [build_runtime(p) for p in profiles]

    for runtime in runtimes:
        p = runtime.profile
        runtime.manus_proc = _start_manus_bridge(
            runtime.shared_state,
            manus_topic=p.manus_topic,
            node_name=f"revo3_{p.side}_manus_bridge",
            debug=args.debug,
        )
        if runtime.manus_proc is None:
            print(f"[Teleop][{p.side}] 无法启动 Manus bridge（topic: {p.manus_topic}）")
            return 1
        print(f"[Teleop][{p.side}] Manus topic: {p.manus_topic}")

    physical_mode = not args.sim_only
    if physical_mode:
        try:
            sdk = _import_sdk()

            async def _detect_all():
                return await sdk.auto_detect(scan_all=True, port=None, protocol=None)

            loop = asyncio.new_event_loop()
            try:
                asyncio.set_event_loop(loop)
                detected = loop.run_until_complete(_detect_all())
            finally:
                asyncio.set_event_loop(None)
                loop.close()
        except Exception as e:
            print(f"[Teleop] SDK auto-detect 不可用: {e}")
            detected = []

        idx = 0
        for runtime in runtimes:
            dev = None
            if runtime.profile.port is None and idx < len(detected):
                dev = detected[idx]
                idx += 1
            start_hardware_loop(runtime, dev)
    else:
        for runtime in runtimes:
            threading.Thread(
                target=_run_mujoco_viewer,
                args=(runtime.profile.model_path, runtime.shared_state),
                daemon=True,
            ).start()

    print(f"[Teleop] 启动完成: left={args.left}, right={args.right}")
    print(f"[Teleop] retarget_hz={args.retarget_hz:.1f}, hardware_hz={args.hardware_hz:.1f}, sim_only={args.sim_only}")

    stop_event = threading.Event()
    if args.plot:
        # matplotlib must own the main thread; push the retarget loop to a background thread.
        retarget_thread = threading.Thread(
            target=_run_retarget_loop,
            args=(runtimes, physical_mode, args.debug, stop_event),
            daemon=True,
        )
        retarget_thread.start()
        try:
            _run_visualization(runtimes, history_seconds=args.plot_history)
        finally:
            stop_event.set()
    else:
        _run_retarget_loop(runtimes, physical_mode, args.debug, stop_event)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
