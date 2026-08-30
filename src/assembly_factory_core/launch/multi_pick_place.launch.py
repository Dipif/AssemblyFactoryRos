import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


CORE_PACKAGE = "assembly_factory_core"
MOVEIT_PACKAGE = "assembly_factory_panda_moveit_config"


def create_nodes(context):
    config_file = LaunchConfiguration("config_file").perform(context)

    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    arms = config.get("arms", [])
    nodes = []

    for arm in arms:
        if not arm.get("enabled", True):
            continue

        robot_name = str(arm["name"])

        pick = arm["pick"]
        place = arm["place"]
        tool_orientation = arm["tool_orientation"]

        object_config = arm["object"]
        object_orientation = object_config["orientation"]

        moveit_config = (
            MoveItConfigsBuilder(
                "panda",
                package_name=MOVEIT_PACKAGE,
            )
            .robot_description(
                file_path="config/panda.urdf.xacro",
                mappings={"robot_name": robot_name},
            )
            .to_moveit_configs()
        )

        parameters = {
            "pick_x": float(pick["x"]),
            "pick_y": float(pick["y"]),
            "pick_z": float(pick["z"]),

            "place_x": float(place["x"]),
            "place_y": float(place["y"]),
            "place_z": float(place["z"]),

            "hover_z": float(arm.get("hover_z", 0.30)),
            "grasp_offset_z": float(
                arm.get("grasp_offset_z", 0.105)
            ),

            "qx": float(tool_orientation["x"]),
            "qy": float(tool_orientation["y"]),
            "qz": float(tool_orientation["z"]),
            "qw": float(tool_orientation["w"]),

            "object_id": str(object_config["id"]),
            "object_size": float(object_config["size"]),

            "object_qx": float(object_orientation["x"]),
            "object_qy": float(object_orientation["y"]),
            "object_qz": float(object_orientation["z"]),
            "object_qw": float(object_orientation["w"]),
        }

        nodes.append(
            Node(
                package=CORE_PACKAGE,
                executable="pick_place",
                name="pick_place",
                namespace=robot_name,
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    parameters,
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    (
                        "joint_states",
                        "joint_state_broadcaster/joint_states",
                    ),
                ],
            )
        )

    return nodes


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory(CORE_PACKAGE),
        "config",
        "arms.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="N-arm pick-and-place configuration file",
            ),
            OpaqueFunction(function=create_nodes),
        ]
    )