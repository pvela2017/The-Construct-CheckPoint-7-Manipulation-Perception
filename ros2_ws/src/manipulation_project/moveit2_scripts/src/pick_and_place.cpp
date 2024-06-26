#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =  move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =  move_group_gripper.getCurrentState()->getJointModelGroup(PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  
  current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  move_group_arm.setNamedTarget("pregrasp");
  move_group_arm.move();

  // Open Gripper
  RCLCPP_INFO(LOGGER, "Open Gripper!");
  move_group_gripper.setNamedTarget("gripper_open");
  move_group_gripper.move();

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");
  geometry_msgs::msg::Pose target_pose1;
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.35;
  target_pose1.orientation.x = 0.721;
  target_pose1.orientation.y = -0.693;
  target_pose1.orientation.z = -0.018;
  target_pose1.orientation.w = -0.006;

  target_pose1.position.z -= 0.075;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.075;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Close Gripper
  RCLCPP_INFO(LOGGER, "Close Gripper!");
  move_group_gripper.setNamedTarget("gripper_close");
  move_group_gripper.move();

  // Retreat
  RCLCPP_INFO(LOGGER, "Retreat from object!");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.075;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.075;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);

  // Place
  RCLCPP_INFO(LOGGER, "Rotating Arm");
  move_group_arm.setNamedTarget("place");
  move_group_arm.move();

  // Open Gripper
  RCLCPP_INFO(LOGGER, "Release Object!");
  move_group_gripper.setNamedTarget("gripper_open");
  move_group_gripper.move();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}