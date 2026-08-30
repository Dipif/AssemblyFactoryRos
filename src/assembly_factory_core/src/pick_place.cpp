#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

  PickPlaceNode() : Node("pick_place")
  {
    declare_parameter("pick_x", 0.50);
    declare_parameter("pick_y", 0.00);
    declare_parameter("pick_z", 0.025);

    declare_parameter("place_x", 0.40);
    declare_parameter("place_y", 0.25);
    declare_parameter("place_z", 0.025);

    declare_parameter("hover_z", 0.30);
    declare_parameter("grasp_offset_z", 0.105);
    declare_parameter("object_size", 0.05);
    declare_parameter("object_id", std::string("pick_cube"));

    declare_parameter("qx", 1.0);
    declare_parameter("qy", 0.0);
    declare_parameter("qz", 0.0);
    declare_parameter("qw", 0.0);

    declare_parameter("object_qx", 1.0);
    declare_parameter("object_qy", 0.0);
    declare_parameter("object_qz", 0.0);
    declare_parameter("object_qw", 0.0);
  }

  void initialize()
  {
    readParameters();

    auto node = shared_from_this();
    const std::string move_group_namespace = get_namespace();

    MoveGroupInterface::Options arm_options("panda_arm", "robot_description", move_group_namespace);

    MoveGroupInterface::Options hand_options("hand", "robot_description", move_group_namespace);

    arm_ = std::make_unique<MoveGroupInterface>(node, arm_options);

    hand_ = std::make_unique<MoveGroupInterface>(node, hand_options);

    planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(move_group_namespace);

    arm_->setPoseReferenceFrame("panda_link0");
    arm_->setPlanningTime(5.0);
    arm_->setNumPlanningAttempts(10);
    arm_->setMaxVelocityScalingFactor(0.1);
    arm_->setMaxAccelerationScalingFactor(0.1);

    hand_->setPlanningTime(5.0);
    hand_->setNumPlanningAttempts(10);
    hand_->setMaxVelocityScalingFactor(0.1);
    hand_->setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(get_logger(), "Initialized PickPlaceNode in namespace '%s'", move_group_namespace.c_str());
  }

  void start()
  {
    worker_ = std::thread([this]() {
      try
      {
        // main()의 executor가 spin을 시작할 시간을 준다.
        std::this_thread::sleep_for(1s);

        if (!runPickAndPlace())
        {
          RCLCPP_ERROR(get_logger(), "Pick-and-place failed");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Pick-and-place completed");
        }
      }
      catch (const std::exception& error)
      {
        RCLCPP_ERROR(get_logger(), "Exception: %s", error.what());
      }

      rclcpp::shutdown();
    });
  }

  ~PickPlaceNode() override
  {
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

private:
  void readParameters()
  {
    pick_x_ = get_parameter("pick_x").as_double();
    pick_y_ = get_parameter("pick_y").as_double();
    pick_z_ = get_parameter("pick_z").as_double();

    place_x_ = get_parameter("place_x").as_double();
    place_y_ = get_parameter("place_y").as_double();
    place_z_ = get_parameter("place_z").as_double();

    hover_z_ = get_parameter("hover_z").as_double();
    grasp_offset_z_ = get_parameter("grasp_offset_z").as_double();

    object_size_ = get_parameter("object_size").as_double();

    object_id_ = get_parameter("object_id").as_string();

    qx_ = get_parameter("qx").as_double();
    qy_ = get_parameter("qy").as_double();
    qz_ = get_parameter("qz").as_double();
    qw_ = get_parameter("qw").as_double();

    
    object_qx_ = get_parameter("object_qx").as_double();
    object_qy_ = get_parameter("object_qy").as_double();
    object_qz_ = get_parameter("object_qz").as_double();
    object_qw_ = get_parameter("object_qw").as_double();
  }

  geometry_msgs::msg::Pose makeArmPose(double x, double y, double z) const
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    pose.orientation.x = qx_;
    pose.orientation.y = qy_;
    pose.orientation.z = qz_;
    pose.orientation.w = qw_;

    return pose;
  }
  bool executeNamedTarget(MoveGroupInterface& group, const std::string& target)
  {
    group.setStartStateToCurrentState();

    if (!group.setNamedTarget(target))
    {
      RCLCPP_ERROR(get_logger(), "Named target '%s' does not exist", target.c_str());
      return false;
    }

    MoveGroupInterface::Plan plan;

    if (!static_cast<bool>(group.plan(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Planning failed for target '%s'", target.c_str());
      return false;
    }

    if (!static_cast<bool>(group.execute(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Execution failed for target '%s'", target.c_str());
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool executePoseTarget(double x, double y, double z)
  {
    arm_->setStartStateToCurrentState();

    const auto target = makeArmPose(x, y, z);
    arm_->setPoseTarget(target);

    MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));

    arm_->clearPoseTargets();

    if (!planned)
    {
      RCLCPP_ERROR(get_logger(), "Pose planning failed: x=%.3f y=%.3f z=%.3f", x, y, z);
      return false;
    }

    if (!static_cast<bool>(arm_->execute(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Pose execution failed");
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool executeCartesianTarget(double x, double y, double z)
  {
    arm_->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(makeArmPose(x, y, z));

    moveit_msgs::msg::RobotTrajectory trajectory;

    const double fraction = arm_->computeCartesianPath(waypoints, 0.005, 0.0, trajectory, true);

    RCLCPP_INFO(get_logger(), "Cartesian path: %.1f%%", fraction * 100.0);
    if (fraction < 0.99)
    {
      RCLCPP_ERROR(get_logger(), "Cartesian path is incomplete");
      return false;
    }

    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    if (!static_cast<bool>(arm_->execute(plan)))
    {
      RCLCPP_ERROR(get_logger(), "Cartesian execution failed");
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  moveit_msgs::msg::CollisionObject makeCube(double x, double y, double z) const
  {
    moveit_msgs::msg::CollisionObject cube;

    cube.header.frame_id = "panda_link0";
    cube.id = object_id_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

    primitive.dimensions = { object_size_, object_size_, object_size_ };

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = object_qx_;
    pose.orientation.y = object_qy_;
    pose.orientation.z = object_qz_;
    pose.orientation.w = object_qw_;

    cube.primitives.push_back(primitive);
    cube.primitive_poses.push_back(pose);
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;

    return cube;
  }

  bool addCubeToPlanningScene(double x, double y, double z)
  {
    const auto cube = makeCube(x, y, z);

    if (!planning_scene_->applyCollisionObject(cube))
    {
      RCLCPP_ERROR(get_logger(), "Failed to add '%s' to planning scene", object_id_.c_str());
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool attachCube()
  {
    const std::vector<std::string> touch_links = { "panda_hand", "panda_leftfinger", "panda_rightfinger" };

    if (!arm_->attachObject(object_id_, "panda_hand", touch_links))
    {
      RCLCPP_ERROR(get_logger(), "Failed to attach '%s'", object_id_.c_str());
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool detachCube()
  {
    if (!arm_->detachObject(object_id_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to detach '%s'", object_id_.c_str());
      return false;
    }

    std::this_thread::sleep_for(500ms);
    return true;
  }

  bool runPickAndPlace()
  {
    const double pick_grasp_z = pick_z_ + grasp_offset_z_;

    const double place_grasp_z = place_z_ + grasp_offset_z_;

    RCLCPP_INFO(get_logger(), "Adding cube");
    if (!addCubeToPlanningScene(pick_x_, pick_y_, pick_z_))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Opening gripper");
    if (!executeNamedTarget(*hand_, "open"))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Moving above pick position");
    if (!executePoseTarget(pick_x_, pick_y_, hover_z_))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Descending to cube");
    if (!executeCartesianTarget(pick_x_, pick_y_, pick_grasp_z))
    {
      return false;
    }

    // 닫기 전에 attach하여 손가락과 큐브의 접촉 충돌을 허용한다.
    RCLCPP_INFO(get_logger(), "Attaching cube in MoveIt");
    if (!attachCube())
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Closing gripper");
    if (!executeNamedTarget(*hand_, "close"))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Lifting cube");
    if (!executeCartesianTarget(pick_x_, pick_y_, hover_z_))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Moving above place position");
    if (!executePoseTarget(place_x_, place_y_, hover_z_))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Descending to place position");
    if (!executeCartesianTarget(place_x_, place_y_, place_grasp_z))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Opening gripper");
    if (!executeNamedTarget(*hand_, "open"))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Detaching cube in MoveIt");
    if (!detachCube())
    {
      return false;
    }

    // 계산 오차와 물리 시뮬레이션 오차에 관계없이
    // Planning Scene에는 사용자가 지정한 놓기 좌표를 기록한다.
    RCLCPP_INFO(get_logger(), "Updating cube world position");

    if (!addCubeToPlanningScene(place_x_, place_y_, place_z_))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Retreating upward");
    if (!executeCartesianTarget(place_x_, place_y_, hover_z_))
    {
      return false;
    }

    return true;
  }

  std::unique_ptr<MoveGroupInterface> arm_;
  std::unique_ptr<MoveGroupInterface> hand_;

  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  std::thread worker_;

  double pick_x_;
  double pick_y_;
  double pick_z_;

  double place_x_;
  double place_y_;
  double place_z_;

  double hover_z_;
  double grasp_offset_z_;
  double object_size_;

  double qx_;
  double qy_;
  double qz_;
  double qw_;

  double object_qx_;
  double object_qy_;
  double object_qz_;
  double object_qw_;

  std::string object_id_;
};
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PickPlaceNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // MoveGroupInterface를 만들기 전에 콜백 처리를 시작한다.
  std::thread executor_thread([&executor]() { executor.spin(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  node->initialize();
  node->start();

  // PickPlaceNode의 작업 스레드가 완료되면
  // rclcpp::shutdown()을 호출하므로 spin도 종료된다.
  executor_thread.join();

  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }

  return 0;
}