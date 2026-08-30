from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    target = LaunchConfiguration("target")
    execute = LaunchConfiguration("execute")
    group = LaunchConfiguration("group")
    target_mode = LaunchConfiguration("target_mode")
    
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    qx = LaunchConfiguration("qx")
    qy = LaunchConfiguration("qy")
    qz = LaunchConfiguration("qz")
    qw = LaunchConfiguration("qw")
    
    object_qx = LaunchConfiguration("object_qx")
    object_qy = LaunchConfiguration("object_qy")
    object_qz = LaunchConfiguration("object_qz")
    object_qw = LaunchConfiguration("object_qw")
    
    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name="assembly_factory_panda_moveit_config",
        )
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"robot_name": robot_name},
        )
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="assembly_factory_core",
        executable="pick_place",
        name="pick_place",
        namespace=robot_name,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "pick_x": LaunchConfiguration("pick_x"),
                "pick_y": LaunchConfiguration("pick_y"),
                "pick_z": LaunchConfiguration("pick_z"),
                "place_x": LaunchConfiguration("place_x"),
                "place_y": LaunchConfiguration("place_y"),
                "place_z": LaunchConfiguration("place_z"),
                "hover_z": LaunchConfiguration("hover_z"),
                "grasp_offset_z": LaunchConfiguration("grasp_offset_z"),
                "object_size": LaunchConfiguration("object_size"),
                "object_id": LaunchConfiguration("object_id"),
                "qx": LaunchConfiguration("qx"),
                "qy": LaunchConfiguration("qy"),
                "qz": LaunchConfiguration("qz"),
                "qw": LaunchConfiguration("qw"),
                "object_qx": LaunchConfiguration("object_qx"),
                "object_qy": LaunchConfiguration("object_qy"),
                "object_qz": LaunchConfiguration("object_qz"),
                "object_qw": LaunchConfiguration("object_qw"),
                
            },
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

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="Franka_01"),

        DeclareLaunchArgument("pick_x", default_value="0.50"),
        DeclareLaunchArgument("pick_y", default_value="0.00"),
        DeclareLaunchArgument("pick_z", default_value="0.025"),

        DeclareLaunchArgument("place_x", default_value="0.40"),
        DeclareLaunchArgument("place_y", default_value="0.25"),
        DeclareLaunchArgument("place_z", default_value="0.025"),

        DeclareLaunchArgument("hover_z", default_value="0.30"),
        DeclareLaunchArgument("grasp_offset_z", default_value="0.105"),

        DeclareLaunchArgument("object_size", default_value="0.05"),
        DeclareLaunchArgument("object_id", default_value="pick_cube"),
        
        DeclareLaunchArgument("qx", default_value="1.0"),
        DeclareLaunchArgument("qy", default_value="0.0"),
        DeclareLaunchArgument("qz", default_value="0.0"),
        DeclareLaunchArgument("qw", default_value="0.0"),
        
        DeclareLaunchArgument("object_qx", default_value="0.0"),
        DeclareLaunchArgument("object_qy", default_value="0.0"),
        DeclareLaunchArgument("object_qz", default_value="0.0"),
        DeclareLaunchArgument("object_qw", default_value="1.0"),
        pick_place_node,
    ])