#!/usr/bin/env python3
import os
from typing import Optional

import numpy as np
import pybullet as p
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from manus_ros2_msgs.msg import ManusGlove
except Exception as exc:
    ManusGlove = None
    MANUS_IMPORT_ERROR = exc
else:
    MANUS_IMPORT_ERROR = None


class EMAFilter:
    def __init__(self, alpha: float = 0.5):
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self.last_val: Optional[np.ndarray] = None

    def update(self, val: np.ndarray) -> np.ndarray:
        val = np.asarray(val, dtype=float)
        if self.last_val is None:
            self.last_val = val.copy()
        else:
            self.last_val = self.alpha * val + (1.0 - self.alpha) * self.last_val
        return self.last_val.copy()


class LeapDirectControl(Node):
    def __init__(self):
        super().__init__("leap_direct_ctrl")
        self._declare_params()
        self._load_params()
        self.manus_enabled = ManusGlove is not None

        self.filter = EMAFilter(alpha=self.ema_alpha)
        self.cmd_joints = np.zeros(16, dtype=float)
        self._joint_lower = np.full(16, -np.pi, dtype=float)
        self._joint_upper = np.full(16, np.pi, dtype=float)
        self._last_rx_time = None
        self._stale_warned = False

        self._connect_pybullet()
        self._load_hand_urdf()
        self._init_debug_targets()

        self.pub_hand = self.create_publisher(JointState, self.cmd_topic, 20)
        self.sub_angles = []
        if self.manus_enabled:
            self.sub_angles = [
                self.create_subscription(ManusGlove, topic, self.cb_angle, 20)
                for topic in self.manus_topics
            ]
        else:
            self.get_logger().warning(
                "manus_ros2_msgs is not available; Manus topic subscriptions are disabled. "
                "Glove joint and short-pose fallback modes can still run."
            )
        self.sub_glove_joints = None
        if self.use_glove_joint_fallback and self.glove_joint_topic:
            self.sub_glove_joints = self.create_subscription(
                JointState, self.glove_joint_topic, self.cb_glove_joint, 20
            )
        self.sub_glove_short = None
        if self.use_thumb_ik and self.glove_short_topic:
            self.sub_glove_short = self.create_subscription(
                PoseArray, self.glove_short_topic, self.cb_glove_short, 10
            )
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"Leap teleop started. manus_topics={self.manus_topics}, cmd_topic={self.cmd_topic}, "
            f"target_side={self.target_side}, target_glove_id={self.target_glove_id}, use_sim={self.use_sim}, "
            f"glove_fallback={self.use_glove_joint_fallback}({self.glove_joint_topic}), "
            f"glove_parse={self.glove_joint_parse_mode}, finger_order={self.glove_joint_finger_order}, "
            f"thumb_ik={self.use_thumb_ik}({self.glove_short_topic}:{self.thumb_short_mid_index}/{self.thumb_short_tip_index}), "
            f"publish_swap={self.publish_swap_mcp_spread_order}, middle_spread={self.use_middle_spread}, "
            f"zero_calib={self.auto_zero_calibration}"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("is_left", False)
        self.declare_parameter("isLeft", False)  # backward compatibility
        self.declare_parameter("use_sim", True)
        self.declare_parameter("manus_topic", "/manus_glove_0")
        self.declare_parameter("manus_topics", ["/manus_glove_0", "/manus_glove_1"])
        self.declare_parameter("use_glove_joint_fallback", True)
        self.declare_parameter("glove_joint_topic", "/glove/r_joints")
        self.declare_parameter("glove_joint_parse_mode", "legacy16")
        self.declare_parameter("glove_joint_finger_order", "thumb,index,middle,ring,pinky")
        self.declare_parameter("use_thumb_ik", True)
        self.declare_parameter("glove_short_topic", "/glove/r_short")
        self.declare_parameter("thumb_short_mid_index", 0)
        self.declare_parameter("thumb_short_tip_index", 1)
        self.declare_parameter("thumb_ik_scale", 1.6)
        self.declare_parameter("thumb_ik_x_scale", 1.15)
        self.declare_parameter("thumb_ik_y_scale", 1.0)
        self.declare_parameter("thumb_ik_z_scale", -1.0)
        self.declare_parameter("thumb_target_ema_alpha", 0.35)
        # A lower blend keeps the mapped posture meaningful while still stabilizing thumb IK.
        self.declare_parameter("thumb_ik_blend", 0.55)
        self.declare_parameter("thumb_ik_max_iters", 40)
        self.declare_parameter("thumb_use_mid_as_secondary", False)
        self.declare_parameter("thumb_ik_update_hz", 20.0)
        self.declare_parameter("thumb_ik_target_deadband_m", 0.0035)
        self.declare_parameter("thumb_ik_q_alpha", 0.20)

        self.declare_parameter("auto_zero_calibration", True)
        self.declare_parameter("zero_sample_count", 50)
        self.declare_parameter("swap_mcp_spread_order", True)
        self.declare_parameter("publish_swap_mcp_spread_order", True)
        self.declare_parameter("use_middle_spread", False)
        self.declare_parameter("cmd_topic", "")
        self.declare_parameter("target_side", "right")
        self.declare_parameter("target_glove_id", -1)
        self.declare_parameter("publish_rate_hz", 120.0)
        self.declare_parameter("command_timeout_s", 0.25)
        # Output smoothing (for stability and better teleop feel).
        self.declare_parameter("ema_alpha", 0.25)
        self.declare_parameter("max_delta_rad", 0.03)
        self.declare_parameter("input_deadzone_deg", 0.5)
        self.declare_parameter("spread_deadzone_deg", 3.0)
        self.declare_parameter("input_angle_unit", "auto")  # auto|deg|rad
        self.declare_parameter("debug_stats", True)
        self.declare_parameter("output_scale", 1.0)
        self.declare_parameter("debug_pb_targets", False)
        self.declare_parameter("debug_pb_target_radius", 0.008)
        self.declare_parameter("use_dip_from_pip", True)
        self.declare_parameter("dip_from_pip_ratio", 0.85)
        self.declare_parameter("use_pip_from_mcp_fallback", True)
        self.declare_parameter("pip_from_mcp_ratio", 0.68)
        self.declare_parameter("pip_from_mcp_min_ratio", 0.25)
        self.declare_parameter("thumb_use_dip_coupling", True)
        self.declare_parameter("thumb_pip_from_mcp_ratio", 0.72)
        self.declare_parameter("thumb_pip_from_mcp_min_ratio", 0.30)
        self.declare_parameter("thumb_dip_from_pip_ratio", 0.80)

        self.declare_parameter("finger_gain", 1.05)
        self.declare_parameter("spread_gain", 0.20)
        self.declare_parameter("thumb_spread_gain", 1.35)
        self.declare_parameter("thumb_mcp_gain", 1.80)
        self.declare_parameter("thumb_pip_gain", 1.95)
        self.declare_parameter("thumb_dip_gain", 1.70)
        self.declare_parameter("thumb_spread_sign", 1.0)
        self.declare_parameter("thumb_flex_sign", 1.0)
        # Default signs here are tuned for the right hand. If you use the left hand, set them via ROS params.
        self.declare_parameter("index_spread_sign", -1.0)
        self.declare_parameter("middle_spread_sign", -1.0)
        self.declare_parameter("index_flex_sign", 1.0)
        self.declare_parameter("middle_flex_sign", 1.0)
        self.declare_parameter("ring_flex_sign", 1.0)
        self.declare_parameter("index_spread_bias_deg", 0.0)
        self.declare_parameter("ring_spread_bias_deg", 0.0)
        self.declare_parameter("ring_spread_sign", 1.0)

    def _load_params(self) -> None:
        is_left_new = bool(self.get_parameter("is_left").value)
        is_left_old = bool(self.get_parameter("isLeft").value)
        self.is_left = is_left_new or is_left_old
        self.use_sim = bool(self.get_parameter("use_sim").value)
        self.manus_topic = str(self.get_parameter("manus_topic").value)
        raw_topics = list(self.get_parameter("manus_topics").value)
        self.manus_topics = [str(t).strip() for t in raw_topics if str(t).strip()]
        if self.manus_topic and self.manus_topic not in self.manus_topics:
            self.manus_topics.append(self.manus_topic)
        if not self.manus_topics:
            self.manus_topics = ["/manus_glove_0"]
        self.use_glove_joint_fallback = bool(self.get_parameter("use_glove_joint_fallback").value)
        self.glove_joint_topic = str(self.get_parameter("glove_joint_topic").value).strip()
        self.glove_joint_parse_mode = str(self.get_parameter("glove_joint_parse_mode").value).strip().lower()
        if self.glove_joint_parse_mode not in ("legacy16", "block20", "auto"):
            self.glove_joint_parse_mode = "legacy16"
        raw_finger_order = str(self.get_parameter("glove_joint_finger_order").value).strip().lower()
        self.glove_joint_finger_order = self._parse_glove_joint_finger_order(raw_finger_order)
        self.use_thumb_ik = bool(self.get_parameter("use_thumb_ik").value)
        self.glove_short_topic = str(self.get_parameter("glove_short_topic").value).strip()
        self.thumb_short_mid_index = int(self.get_parameter("thumb_short_mid_index").value)
        self.thumb_short_tip_index = int(self.get_parameter("thumb_short_tip_index").value)
        self.thumb_ik_scale = float(max(self.get_parameter("thumb_ik_scale").value, 0.01))
        self.thumb_ik_x_scale = float(self.get_parameter("thumb_ik_x_scale").value)
        self.thumb_ik_y_scale = float(self.get_parameter("thumb_ik_y_scale").value)
        self.thumb_ik_z_scale = float(self.get_parameter("thumb_ik_z_scale").value)
        self.thumb_target_ema_alpha = float(np.clip(self.get_parameter("thumb_target_ema_alpha").value, 0.0, 1.0))
        self.thumb_ik_blend = float(np.clip(self.get_parameter("thumb_ik_blend").value, 0.0, 1.0))
        self.thumb_ik_max_iters = int(max(self.get_parameter("thumb_ik_max_iters").value, 10))
        self.thumb_use_mid_as_secondary = bool(self.get_parameter("thumb_use_mid_as_secondary").value)
        self.thumb_ik_update_hz = float(max(self.get_parameter("thumb_ik_update_hz").value, 1.0))
        self.thumb_ik_target_deadband_m = float(max(self.get_parameter("thumb_ik_target_deadband_m").value, 0.0))
        self.thumb_ik_q_alpha = float(np.clip(self.get_parameter("thumb_ik_q_alpha").value, 0.0, 1.0))

        self.auto_zero_calibration = bool(self.get_parameter("auto_zero_calibration").value)
        self.zero_sample_count = int(max(self.get_parameter("zero_sample_count").value, 10))
        self.swap_mcp_spread_order = bool(self.get_parameter("swap_mcp_spread_order").value)
        self.publish_swap_mcp_spread_order = bool(self.get_parameter("publish_swap_mcp_spread_order").value)
        self.use_middle_spread = bool(self.get_parameter("use_middle_spread").value)
        self.target_side = str(self.get_parameter("target_side").value).strip().lower()
        self.target_glove_id = int(self.get_parameter("target_glove_id").value)

        cmd_topic = str(self.get_parameter("cmd_topic").value).strip()
        if not cmd_topic:
            cmd_topic = "/leaphand_node/cmd_allegro_left" if self.is_left else "/leaphand_node/cmd_allegro_right"
        self.cmd_topic = cmd_topic

        self.publish_rate_hz = float(max(self.get_parameter("publish_rate_hz").value, 1.0))
        self.command_timeout_s = float(max(self.get_parameter("command_timeout_s").value, 0.05))
        self.ema_alpha = float(np.clip(self.get_parameter("ema_alpha").value, 0.0, 1.0))
        self.max_delta_rad = float(max(self.get_parameter("max_delta_rad").value, 0.0))
        self.input_deadzone_deg = float(max(self.get_parameter("input_deadzone_deg").value, 0.0))
        self.spread_deadzone_deg = float(max(self.get_parameter("spread_deadzone_deg").value, 0.0))
        self.input_angle_unit = str(self.get_parameter("input_angle_unit").value).strip().lower()
        if self.input_angle_unit not in ("auto", "deg", "rad"):
            self.input_angle_unit = "auto"
        self.debug_stats = bool(self.get_parameter("debug_stats").value)
        self.output_scale = float(max(self.get_parameter("output_scale").value, 0.0))
        self.debug_pb_targets = bool(self.get_parameter("debug_pb_targets").value)
        self.debug_pb_target_radius = float(max(self.get_parameter("debug_pb_target_radius").value, 0.001))
        self.use_dip_from_pip = bool(self.get_parameter("use_dip_from_pip").value)
        self.dip_from_pip_ratio = float(np.clip(self.get_parameter("dip_from_pip_ratio").value, 0.0, 1.5))
        self.use_pip_from_mcp_fallback = bool(self.get_parameter("use_pip_from_mcp_fallback").value)
        self.pip_from_mcp_ratio = float(np.clip(self.get_parameter("pip_from_mcp_ratio").value, 0.0, 1.5))
        self.pip_from_mcp_min_ratio = float(np.clip(self.get_parameter("pip_from_mcp_min_ratio").value, 0.0, 2.0))
        self.thumb_use_dip_coupling = bool(self.get_parameter("thumb_use_dip_coupling").value)
        self.thumb_pip_from_mcp_ratio = float(np.clip(self.get_parameter("thumb_pip_from_mcp_ratio").value, 0.0, 1.8))
        self.thumb_pip_from_mcp_min_ratio = float(np.clip(self.get_parameter("thumb_pip_from_mcp_min_ratio").value, 0.0, 2.0))
        self.thumb_dip_from_pip_ratio = float(np.clip(self.get_parameter("thumb_dip_from_pip_ratio").value, 0.0, 1.8))

        self.finger_gain = float(self.get_parameter("finger_gain").value)
        self.spread_gain = float(self.get_parameter("spread_gain").value)
        self.thumb_spread_gain = float(self.get_parameter("thumb_spread_gain").value)
        self.thumb_mcp_gain = float(self.get_parameter("thumb_mcp_gain").value)
        self.thumb_pip_gain = float(self.get_parameter("thumb_pip_gain").value)
        self.thumb_dip_gain = float(self.get_parameter("thumb_dip_gain").value)
        self.thumb_spread_sign = float(self.get_parameter("thumb_spread_sign").value)
        self.thumb_flex_sign = float(self.get_parameter("thumb_flex_sign").value)
        self.index_spread_sign = float(self.get_parameter("index_spread_sign").value)
        self.middle_spread_sign = float(self.get_parameter("middle_spread_sign").value)
        self.index_flex_sign = float(self.get_parameter("index_flex_sign").value)
        self.middle_flex_sign = float(self.get_parameter("middle_flex_sign").value)
        self.ring_flex_sign = float(self.get_parameter("ring_flex_sign").value)
        self.index_spread_bias_deg = float(self.get_parameter("index_spread_bias_deg").value)
        self.ring_spread_bias_deg = float(self.get_parameter("ring_spread_bias_deg").value)
        self.ring_spread_sign = float(self.get_parameter("ring_spread_sign").value)

        self._rx_total = 0
        self._rx_used = 0
        self._rx_rejected = 0
        self._rx_manus = 0
        self._rx_glove = 0
        self._rx_short = 0
        # Internal tracking for auto unit detection; prefer rad until we see values clearly in degrees.
        self._last_unit_mode = "rad"
        self._last_side_raw = ""
        self._stats_timer_s = 0.0
        self._latest_ang = None

        self._zero_done = not self.auto_zero_calibration
        self._zero_keys = [
            "IndexSpread",
            "IndexMCPStretch",
            "IndexPIPStretch",
            "IndexDIPStretch",
            "MiddleSpread",
            "MiddleMCPStretch",
            "MiddlePIPStretch",
            "MiddleDIPStretch",
            "RingSpread",
            "RingMCPStretch",
            "RingPIPStretch",
            "RingDIPStretch",
            "ThumbMCPSpread",
            "ThumbMCPStretch",
            "ThumbPIPStretch",
            "ThumbDIPStretch",
        ]
        self._zero_buf = {k: [] for k in self._zero_keys}
        self._zero_offset = {k: 0.0 for k in self._zero_keys}
        self._thumb_tip_target = None
        self._thumb_mid_target = None
        self._thumb_tip_filter = EMAFilter(alpha=self.thumb_target_ema_alpha)
        self._thumb_mid_filter = EMAFilter(alpha=self.thumb_target_ema_alpha)
        self._last_thumb_ik_time_s = 0.0
        self._last_thumb_ik_target = None
        self._last_thumb_ik_q = None

        self._pb_dbg_thumb_tip = None
        self._pb_dbg_thumb_mid = None

    def _connect_pybullet(self) -> None:
        self._pb_client = p.connect(p.GUI if self.use_sim else p.DIRECT)
        if self._pb_client < 0:
            raise RuntimeError("PyBullet 连接失败。")
        if self.use_sim:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._pb_client)
        p.setGravity(0, 0, 0, physicsClientId=self._pb_client)
        p.setRealTimeSimulation(0, physicsClientId=self._pb_client)

    def _init_debug_targets(self) -> None:
        if not self.use_sim or not self.debug_pb_targets:
            return
        try:
            r = float(self.debug_pb_target_radius)
            vis_tip = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=r,
                rgbaColor=[1.0, 0.2, 0.2, 0.9],
                physicsClientId=self._pb_client,
            )
            vis_mid = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=r,
                rgbaColor=[0.2, 1.0, 0.2, 0.9],
                physicsClientId=self._pb_client,
            )
            self._pb_dbg_thumb_tip = p.createMultiBody(
                baseMass=0.0,
                baseVisualShapeIndex=vis_tip,
                basePosition=[0.3, 0.3, 0.3],
                physicsClientId=self._pb_client,
            )
            self._pb_dbg_thumb_mid = p.createMultiBody(
                baseMass=0.0,
                baseVisualShapeIndex=vis_mid,
                basePosition=[0.3, 0.3, 0.3],
                physicsClientId=self._pb_client,
            )
            for bid in (self._pb_dbg_thumb_tip, self._pb_dbg_thumb_mid):
                p.setCollisionFilterGroupMask(bid, -1, 0, 0, physicsClientId=self._pb_client)
            self.get_logger().info("PyBullet debug targets enabled (thumb tip/mid).")
        except Exception as exc:
            self.get_logger().warning(f"Failed to init PyBullet debug targets: {exc}")
            self._pb_dbg_thumb_tip = None
            self._pb_dbg_thumb_mid = None

    def _resolve_urdf_path(self) -> str:
        env_path = os.environ.get("LEAP_URDF_PATH", "").strip()
        if env_path and os.path.exists(env_path):
            return env_path

        subdir = "leap_hand_mesh_left" if self.is_left else "leap_hand_mesh_right"
        candidates = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), subdir, "robot_pybullet.urdf"),
        ]
        try:
            share_dir = get_package_share_directory("telekinesis")
            candidates.append(os.path.join(share_dir, subdir, "robot_pybullet.urdf"))
        except PackageNotFoundError:
            pass

        for path in candidates:
            if os.path.exists(path):
                return path
        raise FileNotFoundError(f"找不到 LEAP URDF，尝试路径: {candidates}")

    def _load_hand_urdf(self) -> None:
        urdf_path = self._resolve_urdf_path()
        base_pos = [-0.05, -0.03, -0.125]
        base_quat = p.getQuaternionFromEuler([0.0, 1.57, 1.57])
        self.leap_id = p.loadURDF(
            urdf_path,
            base_pos,
            base_quat,
            useFixedBase=True,
            physicsClientId=self._pb_client,
        )
        self._load_joint_limits()
        self.get_logger().info(f"Loaded URDF: {urdf_path}")

    def _load_joint_limits(self) -> None:
        n_joints = p.getNumJoints(self.leap_id, physicsClientId=self._pb_client)
        dof = min(16, n_joints)
        for idx in range(dof):
            info = p.getJointInfo(self.leap_id, idx, physicsClientId=self._pb_client)
            joint_type = info[2]
            lo = float(info[8])
            hi = float(info[9])
            if joint_type == p.JOINT_REVOLUTE and lo < hi:
                self._joint_lower[idx] = lo
                self._joint_upper[idx] = hi

    def _to_rad(self, value: float) -> float:
        if self._last_unit_mode == "rad":
            return float(value)
        return float(value) * (np.pi / 180.0)

    def _deadzone_rad(self, value_rad: float, *, is_spread: bool = False) -> float:
        threshold_deg = self.spread_deadzone_deg if is_spread else self.input_deadzone_deg
        threshold = float(threshold_deg) * (np.pi / 180.0)
        if abs(value_rad) < threshold:
            return 0.0
        return value_rad

    def _clip_to_limits(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self._joint_lower, self._joint_upper)

    @staticmethod
    def _parse_glove_joint_finger_order(raw: str) -> list[str]:
        # Expected order is 5 comma-separated finger names, each block has [Spread, MCP, PIP, DIP].
        canonical = ["thumb", "index", "middle", "ring", "pinky"]
        alias = {
            "thumb": "thumb",
            "index": "index",
            "middle": "middle",
            "ring": "ring",
            "pinky": "pinky",
            "little": "pinky",
        }
        items = [x.strip().lower() for x in raw.split(",") if x.strip()]
        mapped = [alias.get(x, "") for x in items]
        if len(mapped) == 5 and len(set(mapped)) == 5 and all(x in canonical for x in mapped):
            return mapped
        return canonical

    def _rate_limit(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float)
        if self.max_delta_rad <= 0.0:
            return target
        delta = np.clip(target - self.cmd_joints, -self.max_delta_rad, self.max_delta_rad)
        return self.cmd_joints + delta

    def _transform_short_xyz(self, xyz: np.ndarray) -> np.ndarray:
        x = float(xyz[0]) * self.thumb_ik_scale * self.thumb_ik_x_scale
        y = float(xyz[1]) * self.thumb_ik_scale * self.thumb_ik_y_scale
        z = float(xyz[2]) * self.thumb_ik_scale * self.thumb_ik_z_scale
        return np.asarray([x, y, z], dtype=float)

    def _accumulate_zero_calib(self, ang: dict) -> None:
        if self._zero_done:
            return
        for k in self._zero_keys:
            if k in ang and len(self._zero_buf[k]) < self.zero_sample_count:
                self._zero_buf[k].append(float(ang[k]))
        n = min(len(v) for v in self._zero_buf.values())
        if n >= self.zero_sample_count:
            for k in self._zero_keys:
                self._zero_offset[k] = float(np.median(np.asarray(self._zero_buf[k], dtype=float)))
            self._zero_done = True
            self.get_logger().info(f"Zero calibration done. samples={self.zero_sample_count}")

    def _apply_zero_offset(self, ang: dict) -> dict:
        if not self._zero_done:
            return dict(ang)
        out = {}
        for k, v in ang.items():
            out[k] = float(v) - float(self._zero_offset.get(k, 0.0))
        return out

    def _apply_thumb_ik_to_q(self, q_in: np.ndarray) -> np.ndarray:
        if not self.use_thumb_ik or self._thumb_tip_target is None:
            return q_in
        q = np.asarray(q_in, dtype=float).copy()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._last_thumb_ik_q is not None:
            if now_s - self._last_thumb_ik_time_s < (1.0 / self.thumb_ik_update_hz):
                q[12:16] = (1.0 - self.thumb_ik_blend) * q[12:16] + self.thumb_ik_blend * self._last_thumb_ik_q
                return q
            if self._last_thumb_ik_target is not None:
                dist = float(np.linalg.norm(np.asarray(self._thumb_tip_target) - self._last_thumb_ik_target))
                if dist < self.thumb_ik_target_deadband_m:
                    q[12:16] = (1.0 - self.thumb_ik_blend) * q[12:16] + self.thumb_ik_blend * self._last_thumb_ik_q
                    return q
        try:
            # Warm start IK around the mapped posture.
            for i in range(16):
                p.resetJointState(
                    self.leap_id,
                    i,
                    float(q[i]),
                    physicsClientId=self._pb_client,
                )

            tip = [float(x) for x in self._thumb_tip_target]
            mid = [float(x) for x in self._thumb_mid_target] if self._thumb_mid_target is not None else None
            try:
                if mid is not None and self.thumb_use_mid_as_secondary:
                    ik_solution = p.calculateInverseKinematics2(
                        self.leap_id,
                        [19, 14],
                        [tip, mid],
                        solver=p.IK_DLS,
                        maxNumIterations=self.thumb_ik_max_iters,
                        residualThreshold=1e-4,
                        physicsClientId=self._pb_client,
                    )
                else:
                    raise ValueError("no mid target")
            except Exception:
                ik_solution = p.calculateInverseKinematics(
                    self.leap_id,
                    19,
                    tip,
                    solver=p.IK_DLS,
                    maxNumIterations=self.thumb_ik_max_iters,
                    residualThreshold=1e-4,
                    physicsClientId=self._pb_client,
                )

            ik = np.asarray(ik_solution, dtype=float)
            if ik.shape[0] >= 16:
                q_thumb = ik[12:16]
                if self._last_thumb_ik_q is None:
                    q_thumb_f = q_thumb
                else:
                    q_thumb_f = self.thumb_ik_q_alpha * q_thumb + (1.0 - self.thumb_ik_q_alpha) * self._last_thumb_ik_q
                self._last_thumb_ik_q = np.asarray(q_thumb_f, dtype=float).copy()
                self._last_thumb_ik_target = np.asarray(self._thumb_tip_target, dtype=float).copy()
                self._last_thumb_ik_time_s = now_s
                q[12:16] = (1.0 - self.thumb_ik_blend) * q[12:16] + self.thumb_ik_blend * q_thumb_f
        except Exception as exc:
            self.get_logger().warn(f"Thumb IK failed, fallback to mapping: {exc}")
        return q

    def _update_cmd_from_ang(self, ang: dict) -> None:
        self._latest_ang = dict(ang)
        self._accumulate_zero_calib(ang)
        corrected_ang = self._apply_zero_offset(ang)
        mapped_q = self._map_ergonomics(corrected_ang)
        mapped_q = self._apply_thumb_ik_to_q(mapped_q)
        mapped_q = self._clip_to_limits(mapped_q)
        smoothed_q = self.filter.update(mapped_q)
        self.cmd_joints = self._rate_limit(smoothed_q)
        self._last_rx_time = self.get_clock().now()
        self._stale_warned = False
        self._rx_used += 1

    def _map_ergonomics(self, ang: dict) -> np.ndarray:
        q = np.zeros(16, dtype=float)

        stretch_and_spread_keys = [
            "IndexSpread",
            "IndexMCPStretch",
            "MiddleMCPStretch",
            "RingMCPStretch",
            "ThumbMCPSpread",
            "ThumbMCPStretch",
        ]
        if self.input_angle_unit == "auto":
            # Heuristic: values above ~3.2 strongly indicate degrees (since 3.2 rad ~ 183 deg).
            # Keep previous unit mode otherwise to avoid mode-flapping when values are small.
            vals = [abs(float(ang[k])) for k in stretch_and_spread_keys if k in ang]
            if vals and max(vals) > 3.2:
                self._last_unit_mode = "deg"
        elif self.input_angle_unit == "rad":
            self._last_unit_mode = "rad"
        else:
            self._last_unit_mode = "deg"

        def get_rad(key: str, default: float = 0.0, is_spread: bool = False) -> float:
            v_raw = float(ang.get(key, default))
            v_rad = self._to_rad(v_raw)
            return self._deadzone_rad(v_rad, is_spread=is_spread)

        index_spread = (
            self.index_spread_sign
            * (
                get_rad("IndexSpread", is_spread=True)
                + float(self.index_spread_bias_deg) * (np.pi / 180.0)
            )
            * self.spread_gain
        )
        index_mcp = self.index_flex_sign * get_rad("IndexMCPStretch") * self.finger_gain
        # Internal order follows URDF order: [Spread, MCP, PIP, DIP].
        q[0] = index_spread
        q[1] = index_mcp
        q[2] = self.index_flex_sign * get_rad("IndexPIPStretch") * self.finger_gain
        if self.use_pip_from_mcp_fallback and abs(q[2]) < abs(index_mcp) * self.pip_from_mcp_min_ratio:
            q[2] = index_mcp * self.pip_from_mcp_ratio
        if self.use_dip_from_pip:
            q[3] = q[2] * self.dip_from_pip_ratio
        else:
            q[3] = self.index_flex_sign * get_rad("IndexDIPStretch", float(ang.get("IndexPIPStretch", 0.0))) * self.finger_gain

        middle_spread = self.middle_spread_sign * get_rad("MiddleSpread", is_spread=True) * self.spread_gain
        middle_mcp = self.middle_flex_sign * get_rad("MiddleMCPStretch") * self.finger_gain
        q[4] = middle_spread if self.use_middle_spread else 0.0
        q[5] = middle_mcp
        q[6] = self.middle_flex_sign * get_rad("MiddlePIPStretch") * self.finger_gain
        if self.use_pip_from_mcp_fallback and abs(q[6]) < abs(middle_mcp) * self.pip_from_mcp_min_ratio:
            q[6] = middle_mcp * self.pip_from_mcp_ratio
        if self.use_dip_from_pip:
            q[7] = q[6] * self.dip_from_pip_ratio
        else:
            q[7] = self.middle_flex_sign * get_rad("MiddleDIPStretch", float(ang.get("MiddlePIPStretch", 0.0))) * self.finger_gain

        ring_spread = (
            self.ring_spread_sign
            * (
                get_rad("RingSpread", is_spread=True)
                + float(self.ring_spread_bias_deg) * (np.pi / 180.0)
            )
            * self.spread_gain
        )
        ring_mcp = self.ring_flex_sign * get_rad("RingMCPStretch") * self.finger_gain
        q[8] = ring_spread
        q[9] = ring_mcp
        q[10] = self.ring_flex_sign * get_rad("RingPIPStretch") * self.finger_gain
        if self.use_pip_from_mcp_fallback and abs(q[10]) < abs(ring_mcp) * self.pip_from_mcp_min_ratio:
            q[10] = ring_mcp * self.pip_from_mcp_ratio
        if self.use_dip_from_pip:
            q[11] = q[10] * self.dip_from_pip_ratio
        else:
            q[11] = self.ring_flex_sign * get_rad("RingDIPStretch", float(ang.get("RingPIPStretch", 0.0))) * self.finger_gain

        # Manus ROS2 uses ThumbMCPSpread, not ThumbSpread.
        thumb_spread_raw_default = float(ang.get("ThumbSpread", 0.0))
        thumb_spread = get_rad("ThumbMCPSpread", thumb_spread_raw_default, is_spread=True)
        q[12] = self.thumb_spread_sign * thumb_spread * self.thumb_spread_gain
        q[13] = self.thumb_flex_sign * get_rad("ThumbMCPStretch") * self.thumb_mcp_gain
        q[14] = self.thumb_flex_sign * get_rad("ThumbPIPStretch") * self.thumb_pip_gain
        if abs(q[14]) < abs(q[13]) * self.thumb_pip_from_mcp_min_ratio:
            q[14] = q[13] * self.thumb_pip_from_mcp_ratio
        if self.thumb_use_dip_coupling:
            q[15] = q[14] * self.thumb_dip_from_pip_ratio
        else:
            q[15] = self.thumb_flex_sign * get_rad("ThumbDIPStretch") * self.thumb_dip_gain

        return q

    def _to_publish_order(self, q_urdf: np.ndarray) -> np.ndarray:
        out = np.asarray(q_urdf, dtype=float).copy()
        # Legacy LEAP cmd topic expects [MCP, Spread, PIP, DIP] for the 3 non-thumb fingers.
        if self.publish_swap_mcp_spread_order:
            out[0:2] = out[0:2][::-1]
            out[4:6] = out[4:6][::-1]
            out[8:10] = out[8:10][::-1]
        return out

    def _accept_msg(self, msg: ManusGlove) -> bool:
        if self.target_glove_id >= 0 and int(msg.glove_id) != self.target_glove_id:
            return False
        msg_side = str(msg.side).strip().lower()
        self._last_side_raw = msg_side
        if self.target_side in ("left", "right") and msg_side in ("left", "right"):
            if msg_side != self.target_side:
                return False
        return True

    def cb_angle(self, msg: ManusGlove) -> None:
        self._rx_total += 1
        self._rx_manus += 1
        if not self._accept_msg(msg):
            self._rx_rejected += 1
            return
        ang = {str(ergo.type): float(ergo.value) for ergo in msg.ergonomics}
        if not ang:
            return

        try:
            self._update_cmd_from_ang(ang)
        except Exception as exc:
            self.get_logger().error(f"Mapping error: {exc}")

    def cb_glove_joint(self, msg: JointState) -> None:
        pos = list(msg.position)
        ang = None

        # Previous working parser: consume first 16 values directly.
        if self.glove_joint_parse_mode in ("legacy16", "auto") and len(pos) >= 16:
            ang = {
                "ThumbMCPSpread": float(pos[0]),
                "ThumbMCPStretch": float(pos[1]),
                "ThumbPIPStretch": float(pos[2]),
                "ThumbDIPStretch": float(pos[3]),
                "IndexSpread": float(pos[4]),
                "IndexMCPStretch": float(pos[5]),
                "IndexPIPStretch": float(pos[6]),
                "IndexDIPStretch": float(pos[7]),
                "MiddleSpread": float(pos[8]),
                "MiddleMCPStretch": float(pos[9]),
                "MiddlePIPStretch": float(pos[10]),
                "MiddleDIPStretch": float(pos[11]),
                "RingSpread": float(pos[12]),
                "RingMCPStretch": float(pos[13]),
                "RingPIPStretch": float(pos[14]),
                "RingDIPStretch": float(pos[15]),
            }

        # Optional full 20-dim parser (5 finger blocks: Spread/MCP/PIP/DIP).
        if self.glove_joint_parse_mode == "block20" and len(pos) >= 20:
            starts = {finger: i * 4 for i, finger in enumerate(self.glove_joint_finger_order)}
            t0 = starts.get("thumb", 0)
            i0 = starts.get("index", 4)
            m0 = starts.get("middle", 8)
            r0 = starts.get("ring", 12)
            ang = {
                "ThumbMCPSpread": float(pos[t0 + 0]),
                "ThumbMCPStretch": float(pos[t0 + 1]),
                "ThumbPIPStretch": float(pos[t0 + 2]),
                "ThumbDIPStretch": float(pos[t0 + 3]),
                "IndexSpread": float(pos[i0 + 0]),
                "IndexMCPStretch": float(pos[i0 + 1]),
                "IndexPIPStretch": float(pos[i0 + 2]),
                "IndexDIPStretch": float(pos[i0 + 3]),
                "MiddleSpread": float(pos[m0 + 0]),
                "MiddleMCPStretch": float(pos[m0 + 1]),
                "MiddlePIPStretch": float(pos[m0 + 2]),
                "MiddleDIPStretch": float(pos[m0 + 3]),
                "RingSpread": float(pos[r0 + 0]),
                "RingMCPStretch": float(pos[r0 + 1]),
                "RingPIPStretch": float(pos[r0 + 2]),
                "RingDIPStretch": float(pos[r0 + 3]),
            }

        if ang is None:
            return

        self._rx_total += 1
        self._rx_glove += 1
        try:
            self._update_cmd_from_ang(ang)
        except Exception as exc:
            self.get_logger().error(f"Fallback mapping error: {exc}")

    def cb_glove_short(self, msg: PoseArray) -> None:
        poses = list(msg.poses)
        if len(poses) <= max(self.thumb_short_mid_index, self.thumb_short_tip_index):
            return
        self._rx_short += 1
        try:
            mid_p = poses[self.thumb_short_mid_index].position
            tip_p = poses[self.thumb_short_tip_index].position
            mid_raw = np.asarray([mid_p.x, mid_p.y, mid_p.z], dtype=float)
            tip_raw = np.asarray([tip_p.x, tip_p.y, tip_p.z], dtype=float)

            mid_t = self._transform_short_xyz(mid_raw)
            tip_t = self._transform_short_xyz(tip_raw)
            self._thumb_mid_target = self._thumb_mid_filter.update(mid_t)
            self._thumb_tip_target = self._thumb_tip_filter.update(tip_t)
            if self.use_sim and self.debug_pb_targets and self._pb_dbg_thumb_tip is not None:
                p.resetBasePositionAndOrientation(
                    self._pb_dbg_thumb_tip,
                    [float(x) for x in self._thumb_tip_target],
                    [0.0, 0.0, 0.0, 1.0],
                    physicsClientId=self._pb_client,
                )
            if self.use_sim and self.debug_pb_targets and self._pb_dbg_thumb_mid is not None:
                p.resetBasePositionAndOrientation(
                    self._pb_dbg_thumb_mid,
                    [float(x) for x in self._thumb_mid_target],
                    [0.0, 0.0, 0.0, 1.0],
                    physicsClientId=self._pb_client,
                )
        except Exception as exc:
            self.get_logger().error(f"Short pose parsing failed: {exc}")

    def _on_timer(self) -> None:
        if self._last_rx_time is not None:
            age_s = (self.get_clock().now() - self._last_rx_time).nanoseconds * 1e-9
            if age_s > self.command_timeout_s and not self._stale_warned:
                self.get_logger().warn(
                    f"Manus 数据超时 {age_s:.3f}s（阈值 {self.command_timeout_s:.3f}s），保持上一帧命令。"
                )
                self._stale_warned = True

        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.debug_stats and (now_s - self._stats_timer_s) >= 1.0:
            self._stats_timer_s = now_s
            self.get_logger().info(
                f"rx_total={self._rx_total}, manus={self._rx_manus}, glove={self._rx_glove}, "
                f"short={self._rx_short}, used={self._rx_used}, rejected={self._rx_rejected}, "
                f"last_side={self._last_side_raw}, unit={self._last_unit_mode}, "
                f"zero_done={self._zero_done}, cmd_max={float(np.max(np.abs(self.cmd_joints))):.3f} rad"
            )

        q_out = np.asarray(self.cmd_joints, dtype=float) * float(self.output_scale)
        if self.use_sim:
            for i in range(16):
                p.resetJointState(
                    self.leap_id,
                    i,
                    float(q_out[i]),
                    physicsClientId=self._pb_client,
                )
            p.stepSimulation(physicsClientId=self._pb_client)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(x) for x in self._to_publish_order(q_out)]
        self.pub_hand.publish(msg)

    def destroy_node(self):
        try:
            if hasattr(self, "_pb_client") and self._pb_client >= 0:
                p.disconnect(self._pb_client)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LeapDirectControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
