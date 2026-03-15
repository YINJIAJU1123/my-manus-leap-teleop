from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def _mode_is(mode_value: str) -> IfCondition:
    return IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode_value, "'"]))


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_glove_bridge = LaunchConfiguration("start_glove_bridge")
    start_manus_data_publisher = LaunchConfiguration("start_manus_data_publisher")

    # Shared args forwarded to mode=1 (new pipeline).
    cmd_topic = LaunchConfiguration("cmd_topic")
    target_side = LaunchConfiguration("target_side")
    use_sim = LaunchConfiguration("use_sim")
    use_glove_joint_fallback = LaunchConfiguration("use_glove_joint_fallback")
    glove_joint_topic = LaunchConfiguration("glove_joint_topic")
    glove_joint_parse_mode = LaunchConfiguration("glove_joint_parse_mode")
    use_thumb_ik = LaunchConfiguration("use_thumb_ik")
    glove_short_topic = LaunchConfiguration("glove_short_topic")
    thumb_short_mid_index = LaunchConfiguration("thumb_short_mid_index")
    thumb_short_tip_index = LaunchConfiguration("thumb_short_tip_index")
    mode2_glove_scale = LaunchConfiguration("mode2_glove_scale")
    mode2_thumb_gain = LaunchConfiguration("mode2_thumb_gain")
    mode2_finger_gain = LaunchConfiguration("mode2_finger_gain")
    mode2_thumb_x_scale = LaunchConfiguration("mode2_thumb_x_scale")
    mode2_thumb_y_scale = LaunchConfiguration("mode2_thumb_y_scale")
    mode2_thumb_z_scale = LaunchConfiguration("mode2_thumb_z_scale")
    mode2_thumb_tip_link_index = LaunchConfiguration("mode2_thumb_tip_link_index")
    mode2_thumb_ik_joint_damping = LaunchConfiguration("mode2_thumb_ik_joint_damping")
    mode2_thumb_ik_max_iters = LaunchConfiguration("mode2_thumb_ik_max_iters")
    mode2_thumb_joint_gain = LaunchConfiguration("mode2_thumb_joint_gain")
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    mode1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "leap_manus_teleop.py")),
        launch_arguments={
            "start_manus_data_publisher": start_manus_data_publisher,
            "cmd_topic": cmd_topic,
            "target_side": target_side,
            "use_sim": use_sim,
            "use_glove_joint_fallback": use_glove_joint_fallback,
            "glove_joint_topic": glove_joint_topic,
            "glove_joint_parse_mode": glove_joint_parse_mode,
            "use_thumb_ik": use_thumb_ik,
            "glove_short_topic": glove_short_topic,
            "thumb_short_mid_index": thumb_short_mid_index,
            "thumb_short_tip_index": thumb_short_tip_index,
        }.items(),
        condition=_mode_is("1"),
    )

    # Mode=2 (old pipeline): leap_ik_manus.py
    mode2_manus_pub = Node(
        package="manus_ros2",
        executable="manus_data_publisher",
        name="manus_data_publisher",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    mode,
                    "' == '2' and ('",
                    start_manus_data_publisher,
                    "' == 'true' or '",
                    start_manus_data_publisher,
                    "' == 'True' or '",
                    start_manus_data_publisher,
                    "' == '1')",
                ]
            )
        ),
    )

    mode2_ik = Node(
        package="telekinesis",
        executable="leap_ik_manus",
        name="leap_ik_manus",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim": use_sim},
            {"manus_topic": "/manus_glove_1"},
            {"glove_short_topic": glove_short_topic},
            {"thumb_short_mid_index": thumb_short_mid_index},
            {"thumb_short_tip_index": thumb_short_tip_index},
            {"use_glove_joint_fallback": use_glove_joint_fallback},
            {"glove_joint_topic": glove_joint_topic},
            {"glove_scale": mode2_glove_scale},
            {"thumb_gain": mode2_thumb_gain},
            {"finger_gain": mode2_finger_gain},
            {"thumb_x_scale": mode2_thumb_x_scale},
            {"thumb_y_scale": mode2_thumb_y_scale},
            {"thumb_z_scale": mode2_thumb_z_scale},
            {"thumb_tip_link_index": mode2_thumb_tip_link_index},
            {"thumb_ik_joint_damping": mode2_thumb_ik_joint_damping},
            {"thumb_ik_max_iters": mode2_thumb_ik_max_iters},
            {"thumb_joint_gain": mode2_thumb_joint_gain},
        ],
        condition=_mode_is("2"),
    )

    glove_bridge = Node(
        package="glove",
        executable="read_and_send_zmq",
        name="read_and_send_zmq",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_glove_bridge),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="1",
                description="1=new leap_ik pipeline, 2=legacy leap_ik_manus pipeline",
            ),
            DeclareLaunchArgument("start_glove_bridge", default_value="true"),
            DeclareLaunchArgument("start_manus_data_publisher", default_value="false"),
            DeclareLaunchArgument("cmd_topic", default_value="/leaphand_node/cmd_allegro_right"),
            DeclareLaunchArgument("target_side", default_value="right"),
            DeclareLaunchArgument("use_sim", default_value="true"),
            DeclareLaunchArgument("use_glove_joint_fallback", default_value="true"),
            DeclareLaunchArgument("glove_joint_topic", default_value="/glove/r_joints"),
            DeclareLaunchArgument("glove_joint_parse_mode", default_value="legacy16"),
            DeclareLaunchArgument("use_thumb_ik", default_value="true"),
            DeclareLaunchArgument("glove_short_topic", default_value="/glove/r_short"),
            DeclareLaunchArgument("thumb_short_mid_index", default_value="0"),
            DeclareLaunchArgument("thumb_short_tip_index", default_value="1"),
            DeclareLaunchArgument("mode2_glove_scale", default_value="1.9"),
            DeclareLaunchArgument("mode2_thumb_gain", default_value="2.0"),
            DeclareLaunchArgument("mode2_finger_gain", default_value="1.2"),
            DeclareLaunchArgument("mode2_thumb_x_scale", default_value="1.25"),
            DeclareLaunchArgument("mode2_thumb_y_scale", default_value="1.15"),
            DeclareLaunchArgument("mode2_thumb_z_scale", default_value="-1.20"),
            DeclareLaunchArgument("mode2_thumb_tip_link_index", default_value="19"),
            DeclareLaunchArgument("mode2_thumb_ik_joint_damping", default_value="1.2"),
            DeclareLaunchArgument("mode2_thumb_ik_max_iters", default_value="80"),
            DeclareLaunchArgument("mode2_thumb_joint_gain", default_value="1.35"),
            glove_bridge,
            mode1_launch,
            mode2_manus_pub,
            mode2_ik,
        ]
    )
