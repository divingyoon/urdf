import os

import xacro

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORKSPACE_DIR = "/home/user/rl_ws"
DEFAULT_XACRO_PATH = os.path.join(WORKSPACE_DIR, "urdf", "assemblies", "openarm_modular_dual.xacro")
RVIZ_CONFIG_PATH = os.path.join(
    WORKSPACE_DIR, "urdf", "vendor", "openarm_description", "rviz", "bimanual.rviz"
)


def robot_state_publisher_spawner(
    context: LaunchContext,
    xacro_path,
    right_mount_xyz,
    right_mount_rpy,
    left_mount_xyz,
    left_mount_rpy,
    tesollo_xacro,
):
    xacro_path_value = context.perform_substitution(xacro_path)
    mappings = {}

    right_mount_xyz_value = context.perform_substitution(right_mount_xyz)
    right_mount_rpy_value = context.perform_substitution(right_mount_rpy)
    left_mount_xyz_value = context.perform_substitution(left_mount_xyz)
    left_mount_rpy_value = context.perform_substitution(left_mount_rpy)
    tesollo_xacro_value = context.perform_substitution(tesollo_xacro)

    if right_mount_xyz_value:
        mappings["right_mount_xyz"] = right_mount_xyz_value
    if right_mount_rpy_value:
        mappings["right_mount_rpy"] = right_mount_rpy_value
    if left_mount_xyz_value:
        mappings["left_mount_xyz"] = left_mount_xyz_value
    if left_mount_rpy_value:
        mappings["left_mount_rpy"] = left_mount_rpy_value
    if tesollo_xacro_value:
        mappings["tesollo_xacro"] = tesollo_xacro_value

    robot_description = xacro.process_file(
        xacro_path_value,
        mappings=mappings,
    ).toprettyxml(indent="  ")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    ]


def generate_launch_description():
    xacro_path = LaunchConfiguration("xacro_path")
    right_mount_xyz = LaunchConfiguration("right_mount_xyz")
    right_mount_rpy = LaunchConfiguration("right_mount_rpy")
    left_mount_xyz = LaunchConfiguration("left_mount_xyz")
    left_mount_rpy = LaunchConfiguration("left_mount_rpy")
    tesollo_xacro = LaunchConfiguration("tesollo_xacro")

    robot_state_publisher_loader = OpaqueFunction(
        function=robot_state_publisher_spawner,
        args=[
            xacro_path,
            right_mount_xyz,
            right_mount_rpy,
            left_mount_xyz,
            left_mount_rpy,
            tesollo_xacro,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "xacro_path",
                default_value=DEFAULT_XACRO_PATH,
                description="Absolute path to the xacro file to visualize",
            ),
            DeclareLaunchArgument(
                "right_mount_xyz",
                default_value="",
                description="Optional override for fixed joint xyz from openarm_right_link7 to right_base_link",
            ),
            DeclareLaunchArgument(
                "right_mount_rpy",
                default_value="",
                description="Optional override for fixed joint rpy from openarm_right_link7 to right_base_link",
            ),
            DeclareLaunchArgument(
                "left_mount_xyz",
                default_value="",
                description="Optional override for fixed joint xyz from openarm_left_link7 to left hand",
            ),
            DeclareLaunchArgument(
                "left_mount_rpy",
                default_value="",
                description="Optional override for fixed joint rpy from openarm_left_link7 to left hand",
            ),
            DeclareLaunchArgument(
                "tesollo_xacro",
                default_value="",
                description="Optional override for Tesollo right-hand xacro path",
            ),
            robot_state_publisher_loader,
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {
                        "zeros": {
                            "openarm_left_finger_joint1": 0.02,
                        }
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", RVIZ_CONFIG_PATH],
                output="screen",
            ),
        ]
    )
