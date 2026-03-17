from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Common runtime args you may want to tweak during debugging.
    is_left = LaunchConfiguration("is_left")
    use_sim = LaunchConfiguration("use_sim")
    input_angle_unit = LaunchConfiguration("input_angle_unit")
    publish_swap = LaunchConfiguration("publish_swap_mcp_spread_order")
    glove_parse_mode = LaunchConfiguration("glove_joint_parse_mode")
    glove_finger_order = LaunchConfiguration("glove_joint_finger_order")
    thumb_short_mid_index = LaunchConfiguration("thumb_short_mid_index")
    thumb_short_tip_index = LaunchConfiguration("thumb_short_tip_index")
    ema_alpha = LaunchConfiguration("ema_alpha")
    max_delta_rad = LaunchConfiguration("max_delta_rad")
    finger_gain = LaunchConfiguration("finger_gain")
    spread_gain = LaunchConfiguration("spread_gain")
    output_scale = LaunchConfiguration("output_scale")
    debug_pb_targets = LaunchConfiguration("debug_pb_targets")

    # Tuned defaults for RIGHT hand (is_left=false).
    index_spread_sign = LaunchConfiguration("index_spread_sign")
    middle_spread_sign = LaunchConfiguration("middle_spread_sign")
    ring_spread_sign = LaunchConfiguration("ring_spread_sign")

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_left", default_value="false"),
            DeclareLaunchArgument("use_sim", default_value="true"),
            # Set to "rad" or "deg" if you know the unit; otherwise keep "auto".
            DeclareLaunchArgument("input_angle_unit", default_value="auto"),
            DeclareLaunchArgument("publish_swap_mcp_spread_order", default_value="true"),
            DeclareLaunchArgument("glove_joint_parse_mode", default_value="legacy16"),
            DeclareLaunchArgument("glove_joint_finger_order", default_value="thumb,index,middle,ring,pinky"),
            # NOTE: read_and_send_zmq.py currently publishes short skeleton in order: pinky, thumb, index, ring, middle.
            # So the thumb indices are 2 (mid) and 3 (tip).
            DeclareLaunchArgument("thumb_short_mid_index", default_value="2"),
            DeclareLaunchArgument("thumb_short_tip_index", default_value="3"),
            DeclareLaunchArgument("ema_alpha", default_value="0.25"),
            DeclareLaunchArgument("max_delta_rad", default_value="0.03"),
            DeclareLaunchArgument("finger_gain", default_value="1.0"),
            DeclareLaunchArgument("spread_gain", default_value="0.20"),
            DeclareLaunchArgument("output_scale", default_value="1.0"),
            DeclareLaunchArgument("debug_pb_targets", default_value="false"),
            DeclareLaunchArgument("index_spread_sign", default_value="-1.0"),
            DeclareLaunchArgument("middle_spread_sign", default_value="-1.0"),
            DeclareLaunchArgument("ring_spread_sign", default_value="1.0"),
            Node(
                package="glove",
                executable="read_and_send_zmq",
                name="read_and_send_zmq",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="telekinesis",
                executable="leap_ik",
                name="leap_ik",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"is_left": is_left},
                    {"use_sim": use_sim},
                    {"input_angle_unit": input_angle_unit},
                    {"publish_swap_mcp_spread_order": publish_swap},
                    {"glove_joint_parse_mode": glove_parse_mode},
                    {"glove_joint_finger_order": glove_finger_order},
                    {"thumb_short_mid_index": thumb_short_mid_index},
                    {"thumb_short_tip_index": thumb_short_tip_index},
                    {"ema_alpha": ema_alpha},
                    {"max_delta_rad": max_delta_rad},
                    {"finger_gain": finger_gain},
                    {"spread_gain": spread_gain},
                    {"output_scale": output_scale},
                    {"debug_pb_targets": debug_pb_targets},
                    {"index_spread_sign": index_spread_sign},
                    {"middle_spread_sign": middle_spread_sign},
                    {"ring_spread_sign": ring_spread_sign},
                ],
            ),
        ]
    )
