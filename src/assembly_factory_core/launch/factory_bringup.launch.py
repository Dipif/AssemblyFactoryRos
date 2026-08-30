import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


CORE_PACKAGE = "assembly_factory_core"
MOVEIT_PACKAGE = "assembly_factory_panda_moveit_config"


def launch_setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    task_start_delay = float(
        LaunchConfiguration("task_start_delay").perform(context)
    )

    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file) or {}

    arms = [
        arm
        for arm in config.get("arms", [])
        if arm.get("enabled", True)
    ]

    if not arms:
        raise RuntimeError(
            f"No enabled arms were found in '{config_file}'"
        )

    robot_names = [str(arm["name"]) for arm in arms]

    if len(robot_names) != len(set(robot_names)):
        raise RuntimeError(
            "Duplicate robot names were found in arms.yaml"
        )

    robot_instance_launch = os.path.join(
        get_package_share_directory(MOVEIT_PACKAGE),
        "launch",
        "robot_instance.launch.py",
    )

    multi_pick_place_launch = os.path.join(
        get_package_share_directory(CORE_PACKAGE),
        "launch",
        "multi_pick_place.launch.py",
    )

    actions = []

    for arm in arms:
        robot_name = str(arm["name"])
        use_rviz = str(bool(arm.get("rviz", False))).lower()

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    robot_instance_launch
                ),
                launch_arguments={
                    "robot_name": robot_name,
                    "use_rviz": use_rviz,
                }.items(),
            )
        )

    actions.append(
        TimerAction(
            period=task_start_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        multi_pick_place_launch
                    ),
                    launch_arguments={
                        "config_file": config_file,
                    }.items(),
                )
            ],
        )
    )

    return actions


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
                description="N-arm factory configuration file",
            ),
            DeclareLaunchArgument(
                "task_start_delay",
                default_value="8.0",
                description=(
                    "Seconds to wait for MoveIt and controllers "
                    "before starting pick-and-place"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
