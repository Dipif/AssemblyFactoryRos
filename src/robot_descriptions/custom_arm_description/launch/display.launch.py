from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_model = PathJoinSubstitution(
        [FindPackageShare("custom_arm_description"), "urdf", "custom_arm.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("custom_arm_description"), "rviz", "display.rviz"]
    )

    robot_description = ParameterValue(
        Command(["xacro ", model]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to the custom arm Xacro file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock instead of the system clock",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
