from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manus_topic = LaunchConfiguration("manus_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    target_side = LaunchConfiguration("target_side")
    target_glove_id = LaunchConfiguration("target_glove_id")
    use_sim = LaunchConfiguration("use_sim")
    is_left = LaunchConfiguration("is_left")
    start_manus_data_publisher = LaunchConfiguration("start_manus_data_publisher")
    use_glove_joint_fallback = LaunchConfiguration("use_glove_joint_fallback")
    glove_joint_topic = LaunchConfiguration("glove_joint_topic")
    glove_joint_parse_mode = LaunchConfiguration("glove_joint_parse_mode")
    glove_joint_finger_order = LaunchConfiguration("glove_joint_finger_order")
    use_thumb_ik = LaunchConfiguration("use_thumb_ik")
    glove_short_topic = LaunchConfiguration("glove_short_topic")
    thumb_short_mid_index = LaunchConfiguration("thumb_short_mid_index")
    thumb_short_tip_index = LaunchConfiguration("thumb_short_tip_index")
    auto_zero_calibration = LaunchConfiguration("auto_zero_calibration")
    zero_sample_count = LaunchConfiguration("zero_sample_count")
    swap_mcp_spread_order = LaunchConfiguration("swap_mcp_spread_order")
    thumb_use_mid_as_secondary = LaunchConfiguration("thumb_use_mid_as_secondary")
    thumb_ik_update_hz = LaunchConfiguration("thumb_ik_update_hz")
    thumb_ik_target_deadband_m = LaunchConfiguration("thumb_ik_target_deadband_m")
    thumb_ik_q_alpha = LaunchConfiguration("thumb_ik_q_alpha")
    use_pip_from_mcp_fallback = LaunchConfiguration("use_pip_from_mcp_fallback")
    pip_from_mcp_ratio = LaunchConfiguration("pip_from_mcp_ratio")
    pip_from_mcp_min_ratio = LaunchConfiguration("pip_from_mcp_min_ratio")
    thumb_use_dip_coupling = LaunchConfiguration("thumb_use_dip_coupling")
    thumb_pip_from_mcp_ratio = LaunchConfiguration("thumb_pip_from_mcp_ratio")
    thumb_pip_from_mcp_min_ratio = LaunchConfiguration("thumb_pip_from_mcp_min_ratio")
    thumb_dip_from_pip_ratio = LaunchConfiguration("thumb_dip_from_pip_ratio")
    publish_swap_mcp_spread_order = LaunchConfiguration("publish_swap_mcp_spread_order")
    use_middle_spread = LaunchConfiguration("use_middle_spread")
    spread_gain = LaunchConfiguration("spread_gain")
    thumb_ik_blend = LaunchConfiguration("thumb_ik_blend")
    thumb_mcp_gain = LaunchConfiguration("thumb_mcp_gain")
    thumb_pip_gain = LaunchConfiguration("thumb_pip_gain")
    thumb_dip_gain = LaunchConfiguration("thumb_dip_gain")
    index_spread_sign = LaunchConfiguration("index_spread_sign")
    middle_spread_sign = LaunchConfiguration("middle_spread_sign")
    ring_spread_sign = LaunchConfiguration("ring_spread_sign")
    thumb_spread_sign = LaunchConfiguration("thumb_spread_sign")
    thumb_flex_sign = LaunchConfiguration("thumb_flex_sign")
    index_flex_sign = LaunchConfiguration("index_flex_sign")
    middle_flex_sign = LaunchConfiguration("middle_flex_sign")
    ring_flex_sign = LaunchConfiguration("ring_flex_sign")
    index_spread_bias_deg = LaunchConfiguration("index_spread_bias_deg")
    ring_spread_bias_deg = LaunchConfiguration("ring_spread_bias_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument("manus_topic", default_value="/manus_glove_0"),
            DeclareLaunchArgument("cmd_topic", default_value="/leaphand_node/cmd_allegro_right"),
            DeclareLaunchArgument("target_side", default_value="right"),
            DeclareLaunchArgument("target_glove_id", default_value="-1"),
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("is_left", default_value="false"),
            DeclareLaunchArgument("start_manus_data_publisher", default_value="true"),
            DeclareLaunchArgument("use_glove_joint_fallback", default_value="true"),
            DeclareLaunchArgument("glove_joint_topic", default_value="/glove/r_joints"),
            DeclareLaunchArgument("glove_joint_parse_mode", default_value="legacy16"),
            DeclareLaunchArgument("glove_joint_finger_order", default_value="thumb,index,middle,ring,pinky"),
            DeclareLaunchArgument("use_thumb_ik", default_value="true"),
            DeclareLaunchArgument("glove_short_topic", default_value="/glove/r_short"),
            DeclareLaunchArgument("thumb_short_mid_index", default_value="0"),
            DeclareLaunchArgument("thumb_short_tip_index", default_value="1"),
            DeclareLaunchArgument("auto_zero_calibration", default_value="true"),
            DeclareLaunchArgument("zero_sample_count", default_value="50"),
            DeclareLaunchArgument("swap_mcp_spread_order", default_value="true"),
            DeclareLaunchArgument("publish_swap_mcp_spread_order", default_value="true"),
            DeclareLaunchArgument("use_middle_spread", default_value="false"),
            DeclareLaunchArgument("thumb_use_mid_as_secondary", default_value="false"),
            DeclareLaunchArgument("thumb_ik_update_hz", default_value="15.0"),
            DeclareLaunchArgument("thumb_ik_target_deadband_m", default_value="0.006"),
            DeclareLaunchArgument("thumb_ik_q_alpha", default_value="0.20"),
            DeclareLaunchArgument("use_pip_from_mcp_fallback", default_value="true"),
            DeclareLaunchArgument("pip_from_mcp_ratio", default_value="0.68"),
            DeclareLaunchArgument("pip_from_mcp_min_ratio", default_value="0.25"),
            DeclareLaunchArgument("thumb_use_dip_coupling", default_value="true"),
            DeclareLaunchArgument("thumb_pip_from_mcp_ratio", default_value="0.72"),
            DeclareLaunchArgument("thumb_pip_from_mcp_min_ratio", default_value="0.30"),
            DeclareLaunchArgument("thumb_dip_from_pip_ratio", default_value="0.80"),
            DeclareLaunchArgument("spread_gain", default_value="0.20"),
            DeclareLaunchArgument("thumb_ik_blend", default_value="0.55"),
            DeclareLaunchArgument("thumb_mcp_gain", default_value="1.80"),
            DeclareLaunchArgument("thumb_pip_gain", default_value="1.95"),
            DeclareLaunchArgument("thumb_dip_gain", default_value="1.70"),
            DeclareLaunchArgument("index_spread_sign", default_value="-1.0"),
            DeclareLaunchArgument("middle_spread_sign", default_value="-1.0"),
            DeclareLaunchArgument("ring_spread_sign", default_value="1.0"),
            DeclareLaunchArgument("thumb_spread_sign", default_value="1.0"),
            DeclareLaunchArgument("thumb_flex_sign", default_value="1.0"),
            DeclareLaunchArgument("index_flex_sign", default_value="1.0"),
            DeclareLaunchArgument("middle_flex_sign", default_value="1.0"),
            DeclareLaunchArgument("ring_flex_sign", default_value="1.0"),
            DeclareLaunchArgument("index_spread_bias_deg", default_value="0.0"),
            DeclareLaunchArgument("ring_spread_bias_deg", default_value="0.0"),
            Node(
                package="manus_ros2",
                executable="manus_data_publisher",
                name="manus_data_publisher",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(start_manus_data_publisher),
            ),
            Node(
                package="telekinesis",
                executable="leap_ik",
                name="leap_ik",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"manus_topic": manus_topic},
                    {"cmd_topic": cmd_topic},
                    {"target_side": target_side},
                    {"target_glove_id": target_glove_id},
                    {"use_sim": use_sim},
                    {"is_left": is_left},
                    {"use_glove_joint_fallback": use_glove_joint_fallback},
                    {"glove_joint_topic": glove_joint_topic},
                    {"glove_joint_parse_mode": glove_joint_parse_mode},
                    {"glove_joint_finger_order": glove_joint_finger_order},
                    {"use_thumb_ik": use_thumb_ik},
                    {"glove_short_topic": glove_short_topic},
                    {"thumb_short_mid_index": thumb_short_mid_index},
                    {"thumb_short_tip_index": thumb_short_tip_index},
                    {"auto_zero_calibration": auto_zero_calibration},
                    {"zero_sample_count": zero_sample_count},
                    {"swap_mcp_spread_order": swap_mcp_spread_order},
                    {"publish_swap_mcp_spread_order": publish_swap_mcp_spread_order},
                    {"use_middle_spread": use_middle_spread},
                    {"thumb_use_mid_as_secondary": thumb_use_mid_as_secondary},
                    {"thumb_ik_update_hz": thumb_ik_update_hz},
                    {"thumb_ik_target_deadband_m": thumb_ik_target_deadband_m},
                    {"thumb_ik_q_alpha": thumb_ik_q_alpha},
                    {"use_pip_from_mcp_fallback": use_pip_from_mcp_fallback},
                    {"pip_from_mcp_ratio": pip_from_mcp_ratio},
                    {"pip_from_mcp_min_ratio": pip_from_mcp_min_ratio},
                    {"thumb_use_dip_coupling": thumb_use_dip_coupling},
                    {"thumb_pip_from_mcp_ratio": thumb_pip_from_mcp_ratio},
                    {"thumb_pip_from_mcp_min_ratio": thumb_pip_from_mcp_min_ratio},
                    {"thumb_dip_from_pip_ratio": thumb_dip_from_pip_ratio},
                    {"spread_gain": spread_gain},
                    {"thumb_ik_blend": thumb_ik_blend},
                    {"thumb_mcp_gain": thumb_mcp_gain},
                    {"thumb_pip_gain": thumb_pip_gain},
                    {"thumb_dip_gain": thumb_dip_gain},
                    {"index_spread_sign": index_spread_sign},
                    {"middle_spread_sign": middle_spread_sign},
                    {"ring_spread_sign": ring_spread_sign},
                    {"thumb_spread_sign": thumb_spread_sign},
                    {"thumb_flex_sign": thumb_flex_sign},
                    {"index_flex_sign": index_flex_sign},
                    {"middle_flex_sign": middle_flex_sign},
                    {"ring_flex_sign": ring_flex_sign},
                    {"index_spread_bias_deg": index_spread_bias_deg},
                    {"ring_spread_bias_deg": ring_spread_bias_deg},
                ],
            ),
        ]
    )
