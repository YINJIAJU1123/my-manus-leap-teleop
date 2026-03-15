#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import os

# 尝试导入 Manus 消息，如果没有会报错
try:
    from manus_ros2_msgs.msg import ManusGlove
except ImportError:
    print("错误: 找不到 manus_ros2_msgs。请确保你安装了Manus的ROS2包。")
    # 如果实在没有，这部分代码无法运行直接映射逻辑

'''
Leap Hybrid Controller
策略：
1. 食指(Index)、中指(Middle)、无名指(Ring) -> 直接角度映射 (Direct Mapping)
2. 拇指(Thumb) -> 逆运动学 (IK)
3. 小指(Pinky) -> 丢弃
'''

class LeapHybridControl(Node):
    def __init__(self):
        super().__init__('leap_hybrid_ctrl')
        
        # --- 参数配置 ---
        self.declare_parameter("is_left", False)
        self.declare_parameter("use_sim", True)
        self.declare_parameter("manus_topic", "/manus_glove_1")
        self.declare_parameter("glove_short_topic", "/glove/r_short")
        self.declare_parameter("thumb_short_mid_index", 0)
        self.declare_parameter("thumb_short_tip_index", 1)
        self.declare_parameter("use_glove_joint_fallback", True)
        self.declare_parameter("glove_joint_topic", "/glove/r_joints")
        self.declare_parameter("glove_scale", 1.9)
        self.declare_parameter("thumb_gain", 2.0)
        self.declare_parameter("finger_gain", 1.2)
        self.declare_parameter("thumb_x_scale", 1.25)
        self.declare_parameter("thumb_y_scale", 1.15)
        self.declare_parameter("thumb_z_scale", -1.20)
        self.declare_parameter("thumb_tip_link_index", 19)
        self.declare_parameter("thumb_ik_joint_damping", 1.2)
        self.declare_parameter("thumb_ik_max_iters", 80)
        self.declare_parameter("thumb_joint_gain", 1.35)
        self.is_left = bool(self.get_parameter("is_left").value)
        self.use_sim = bool(self.get_parameter("use_sim").value)  # True: GUI可视化, False: 直接运行
        self.manus_topic = str(self.get_parameter("manus_topic").value)
        self.glove_short_topic = str(self.get_parameter("glove_short_topic").value)
        self.thumb_short_mid_index = int(self.get_parameter("thumb_short_mid_index").value)
        self.thumb_short_tip_index = int(self.get_parameter("thumb_short_tip_index").value)
        self.use_glove_joint_fallback = bool(self.get_parameter("use_glove_joint_fallback").value)
        self.glove_joint_topic = str(self.get_parameter("glove_joint_topic").value)
        
        # 拇指 IK 和直接映射参数
        self.glove_scale = float(self.get_parameter("glove_scale").value)
        self.thumb_gain = float(self.get_parameter("thumb_gain").value)
        self.finger_gain = float(self.get_parameter("finger_gain").value)
        self.thumb_x_scale = float(self.get_parameter("thumb_x_scale").value)
        self.thumb_y_scale = float(self.get_parameter("thumb_y_scale").value)
        self.thumb_z_scale = float(self.get_parameter("thumb_z_scale").value)
        self.thumb_tip_index = int(self.get_parameter("thumb_tip_link_index").value)
        self.thumb_ik_joint_damping = float(self.get_parameter("thumb_ik_joint_damping").value)
        self.thumb_ik_max_iters = int(self.get_parameter("thumb_ik_max_iters").value)
        self.thumb_joint_gain = float(self.get_parameter("thumb_joint_gain").value)

        # --- PyBullet 初始化 (仅用于拇指IK) ---
        p.connect(p.GUI if self.use_sim else p.DIRECT)
        path_src = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
        self.LeapId = p.loadURDF(urdf_path, [-0.05, -0.03, -0.125], p.getQuaternionFromEuler([0, 1.57, 1.57]), useFixedBase=True)
        self.numJoints = p.getNumJoints(self.LeapId)

        # 定义 Leap 的关节索引 (请根据你的 URDF 核对)
        # 假设顺序: [Index_Abd, Index_MCP, Index_PIP, Index_DIP, Middle..., Ring..., Thumb...]
        # Leap 只有 16 个关节
        self.joint_indices = {
            'index': [0, 1, 2, 3],
            'middle': [4, 5, 6, 7],
            'ring': [8, 9, 10, 11],
            'thumb': [12, 13, 14, 15]
        }
        
        # --- ROS 通讯 ---
        self.pub_hand = self.create_publisher(JointState, '/leaphand_node/cmd_allegro_right', 10)
        
        # 订阅手套的坐标数据 (用于拇指 IK)
        self.sub_pos = self.create_subscription(PoseArray, self.glove_short_topic, self.cb_pose, 10)
        
        # 订阅手套的角度数据 (用于其他手指直接映射)
        # 注意：你需要确认这个 topic 名字，通常是 /manus_glove_0 或 /manus/glove_source
        self.sub_angle = self.create_subscription(ManusGlove, self.manus_topic, self.cb_angle, 10)
        self.sub_glove_joint = None
        if self.use_glove_joint_fallback:
            self.sub_glove_joint = self.create_subscription(JointState, self.glove_joint_topic, self.cb_glove_joint, 10)

        # 存储最新的数据
        self.latest_thumb_target = None
        self.latest_angles = {} # 存 Index, Middle, Ring 的角度

        # 最终发送给 Leap 的 16 个关节角度
        self.cmd_joints = np.zeros(16)

        self.get_logger().info(
            f"Leap Hybrid Controller Started. use_sim={self.use_sim}, "
            f"manus_topic={self.manus_topic}, glove_short_topic={self.glove_short_topic}, "
            f"glove_joint_fallback={self.use_glove_joint_fallback}({self.glove_joint_topic}), "
            f"thumb_short_idx={self.thumb_short_mid_index}/{self.thumb_short_tip_index}, "
            f"thumb_scale=({self.glove_scale},{self.thumb_gain},{self.thumb_x_scale},{self.thumb_y_scale},{self.thumb_z_scale}), "
            f"thumb_ik=(tip={self.thumb_tip_index}, damp={self.thumb_ik_joint_damping}, iters={self.thumb_ik_max_iters}, q_gain={self.thumb_joint_gain})"
        )

    def cb_pose(self, msg):
        """处理坐标数据：只提取拇指"""
        # Manus PoseArray索引: 0=Thumb_Mid, 1=Thumb_Tip
        if len(msg.poses) <= max(self.thumb_short_mid_index, self.thumb_short_tip_index):
            return
        thumb_tip_raw = msg.poses[self.thumb_short_tip_index].position
        
        # 简单的坐标转换与缩放
        x = thumb_tip_raw.x * self.glove_scale * self.thumb_gain * self.thumb_x_scale
        y = thumb_tip_raw.y * self.glove_scale * self.thumb_gain * self.thumb_y_scale
        z = thumb_tip_raw.z * self.glove_scale * self.thumb_gain * self.thumb_z_scale
        
        self.latest_thumb_target = [x, y, z]
        
        # 触发一次控制循环
        self.control_loop()

    def cb_angle(self, msg):
        """处理角度数据：提取食指、中指、无名指的弯曲度"""
        # ManusGlove 的 ergonomics 字段包含了所有关节角度
        # 我们需要遍历它找到我们需要的值
        
        angles = {}
        for ergo in msg.ergonomics:
            # 这里的名字对应 Manus SDK 的定义
            # 食指
            if ergo.type == "IndexMCPStretch": angles['index_mcp'] = ergo.value
            if ergo.type == "IndexPIPStretch": angles['index_pip'] = ergo.value
            if ergo.type == "IndexSpread":     angles['index_abd'] = ergo.value # 张开
            
            # 中指
            if ergo.type == "MiddleMCPStretch": angles['middle_mcp'] = ergo.value
            if ergo.type == "MiddlePIPStretch": angles['middle_pip'] = ergo.value
            
            # 无名指
            if ergo.type == "RingMCPStretch": angles['ring_mcp'] = ergo.value
            if ergo.type == "RingPIPStretch": angles['ring_pip'] = ergo.value
            if ergo.type == "RingSpread":     angles['ring_abd'] = ergo.value

        self.latest_angles = angles
        self.control_loop()

    def cb_glove_joint(self, msg):
        """当 Manus topic 没数据时，使用 /glove/r_joints 回退。"""
        pos = list(msg.position)
        if len(pos) < 16:
            return
        self.latest_angles = {
            "index_abd": float(pos[4]),
            "index_mcp": float(pos[5]),
            "index_pip": float(pos[6]),
            "middle_mcp": float(pos[9]),
            "middle_pip": float(pos[10]),
            "ring_abd": float(pos[12]),
            "ring_mcp": float(pos[13]),
            "ring_pip": float(pos[14]),
        }
        self.control_loop()

    def control_loop(self):
        # --- 1. Thumb Control (IK) ---
        if self.latest_thumb_target is not None:
            # 使用 PyBullet 解算拇指
            # 这是一个 "Single Goal IK" (只解指尖)，比之前的 8 点 IK 自由度更高
            damping = [0.1] * self.numJoints
            for i in self.joint_indices['thumb']:
                damping[i] = self.thumb_ik_joint_damping
            ik_solution = p.calculateInverseKinematics(
                self.LeapId,
                self.thumb_tip_index,
                self.latest_thumb_target,
                solver=p.IK_DLS,
                maxNumIterations=self.thumb_ik_max_iters,
                residualThreshold=1e-4,
                jointDamping=damping
            )
            # 假设 IK 返回所有关节，最后 4 个是拇指
            thumb_q = np.asarray(ik_solution[-4:], dtype=float)
            thumb_q[1:4] *= self.thumb_joint_gain
            self.cmd_joints[12:16] = thumb_q

        # --- 2. Finger Control (Direct Mapping) ---
        # 如果没有角度数据，就保持不动或用 IK 的旧值
        if self.latest_angles:
            ang = self.latest_angles
            
            # 辅助函数：将角度(度)转为弧度，并乘系数
            def map_val(val_deg):
                return np.deg2rad(val_deg) * self.finger_gain

            # --- Index ---
            # Joint 0: Abduction (侧摆)
            self.cmd_joints[0] = map_val(ang.get('index_abd', 0)) 
            # Joint 1: MCP (根部弯曲)
            self.cmd_joints[1] = map_val(ang.get('index_mcp', 0))
            # Joint 2: PIP (中间弯曲)
            self.cmd_joints[2] = map_val(ang.get('index_pip', 0))
            # Joint 3: DIP (指尖) -> Leap通常让它跟随PIP，或者也用PIP的数据
            self.cmd_joints[3] = map_val(ang.get('index_pip', 0)) 

            # --- Middle ---
            self.cmd_joints[4] = 0 # 中指通常不能侧摆，设为0
            self.cmd_joints[5] = map_val(ang.get('middle_mcp', 0))
            self.cmd_joints[6] = map_val(ang.get('middle_pip', 0))
            self.cmd_joints[7] = map_val(ang.get('middle_pip', 0))

            # --- Ring ---
            self.cmd_joints[8] = -map_val(ang.get('ring_abd', 0)) # 方向可能相反
            self.cmd_joints[9] = map_val(ang.get('ring_mcp', 0))
            self.cmd_joints[10] = map_val(ang.get('ring_pip', 0))
            self.cmd_joints[11] = map_val(ang.get('ring_pip', 0))

        # --- 3. Publish ---
        # 某些 Leap 安装包要求的关节顺序可能需要反转，保留之前的兼容代码
        final_q = self.cmd_joints.copy()
        # final_q[0:2] = final_q[0:2][::-1] # 如果发现食指左右反了，取消注释

        # 同步到 PyBullet，可视化时才能看到手动起来。
        if self.use_sim:
            for i in range(16):
                p.resetJointState(self.LeapId, i, float(final_q[i]))
            p.stepSimulation()
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [float(x) for x in final_q]
        self.pub_hand.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeapHybridControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
