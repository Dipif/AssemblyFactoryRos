import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from nav2_common.launch import ReplaceString


PACKAGE_NAME = "assembly_factory_panda_moveit_config"


def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")

    controller_manager_name = PathJoinSubstitution(
        ["/", robot_name, "controller_manager"]
    )

    package_share = get_package_share_directory(PACKAGE_NAME)

    ros2_controllers_file = os.path.join(
        package_share,
        "config",
        "ros2_controllers.yaml",
    )

    rviz_config_template = os.path.join(
        package_share,
        "config",
        "moveit.rviz",
    )

    rviz_config_file = ReplaceString(
        source_file=rviz_config_template,
        replacements={
            "__ROBOT_NAMESPACE__": ["/", robot_name],
        },
    )
    moveit_config = (
        MoveItConfigsBuilder(
            "panda",
            package_name=PACKAGE_NAME,
        )
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"robot_name": robot_name},
        )
        .to_moveit_configs()
    )

    # 같은 링크 이름을 사용하는 다른 로봇과 TF 및 joint_states를 분리한다.
    # ros 2 에서 "/" 로 시작하는 경로는 절대 경로를 의미
    # 그렇지 않은 경우 상대 경로를 의미
    # 예) /tf 는 항상 /tf 이나 tf 는 namespace에 의존하여 /robot_name/tf 가 됨
    # remapping은 topic의 이름을 변경하는 방식으로 동작
    tf_remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")
    ]
    moveit_remappings = tf_remappings + [
        ("joint_states", "joint_state_broadcaster/joint_states")
    ]
    control_remappings = tf_remappings + [
        ("/joint_states", "moveit_joint_states")
    ]
    # /tf_static 발행 (remapping: /robot_name/tf_static)
    # transform from world to link0 
    world_to_robot_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_name,
        name="world_to_robot_tf",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "panda_link0",
        ],
        remappings=moveit_remappings,
        output="log",
    )

    # /Franka_01/moveit_joint_states 구독 
    # /tf, /tf_static 발행 (remapping: /robot_name/tf, /robot_name/tf_static)
    # transform from link_n to link_n+1
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        name="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        remappings=moveit_remappings,
        output="screen",
    )

    # /tf, /tf_static 구독 (remapping: /robot_name/tf, /robot_name/tf_static)
    # joint_states 구독 (remapping: /robot_name/moveit_joint_states)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_name,
        name="move_group",
        parameters=[moveit_config.to_dict()],
        remappings=moveit_remappings,
        output="screen",
    )

    # controller manager 실행
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=robot_name,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file,
        ],
        remappings=control_remappings,
        output="screen",
    )

    # controller manager에 joint_static_broadcaster 추가(=Spawn)
    # isaac sim의 /robot_name/joint_states 구독 (...ros2_control.xacro 의 joint_states_topic에 따름)
    # /joint_states 발행 (remapping: /robot_name/moveit_joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
        output="screen",
    )

    # controller manager에 panda_arm_controller 추가(=Spawn)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "panda_arm_controller",
            "--controller-manager",
            controller_manager_name,
            "--param-file",
            ros2_controllers_file,
        ],
        output="screen",
    )

    # controller manager에 panda_hand_controller 추가(=Spawn)
    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_name,
        arguments=[
            "panda_hand_controller",
            "--controller-manager",
            controller_manager_name,
            "--param-file",
            ros2_controllers_file,
        ],
        output="screen",
    )

    # /tf, /tf_static 구독 (remapping: /robot_name/tf, /robot_name/tf_static)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=robot_name,
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        remappings=moveit_remappings,
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_name",
                default_value="Franka_01",
                description="Robot namespace and Isaac Sim robot identifier",
            ),
            world_to_robot_tf,
            robot_state_publisher,
            move_group,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
            rviz,
        ]
    )