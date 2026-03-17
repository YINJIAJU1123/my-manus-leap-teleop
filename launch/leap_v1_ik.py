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

    # Tuned defaults for RIGHT hand (is_left=false).
    index_spread_sign = LaunchConfiguration("index_spread_sign")
    middle_spread_sign = LaunchConfiguration("middle_spread_sign")
    ring_spread_sign = LaunchConfiguration("ring_spread_sign")

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_left", default_value="false"),
            DeclareLaunchArgument("use_sim", default_value="true"),
            # For glove joint fallback, Manus SDK commonly outputs radians. If your values look like degrees, set to "deg".
            DeclareLaunchArgument("input_angle_unit", default_value="rad"),
            DeclareLaunchArgument("publish_swap_mcp_spread_order", default_value="true"),
            DeclareLaunchArgument("glove_joint_parse_mode", default_value="legacy16"),
            DeclareLaunchArgument("glove_joint_finger_order", default_value="thumb,index,middle,ring,pinky"),
            DeclareLaunchArgument("thumb_short_mid_index", default_value="0"),
            DeclareLaunchArgument("thumb_short_tip_index", default_value="1"),
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
                    {"index_spread_sign": index_spread_sign},
                    {"middle_spread_sign": middle_spread_sign},
                    {"ring_spread_sign": ring_spread_sign},
                ],
            ),
        ]
    )
