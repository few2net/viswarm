#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"


// find pseudo inverse jacobian
Eigen::MatrixXd getInvJacobian(Eigen::MatrixXd jacobian)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_of_j(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd& u = svd_of_j.matrixU();
  const Eigen::MatrixXd& v = svd_of_j.matrixV();
  const Eigen::VectorXd& s = svd_of_j.singularValues();

  Eigen::VectorXd sinv = s;
  static const double PINVTOLER = std::numeric_limits<float>::epsilon();
  double maxsv = 0.0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i)
    if (fabs(s(i)) > maxsv)
      maxsv = fabs(s(i));
  for (std::size_t i = 0; i < static_cast<std::size_t>(s.rows()); ++i)
  {
    // Those singular values smaller than a percentage of the maximum singular value are removed
    if (fabs(s(i)) > maxsv * PINVTOLER)
      sinv(i) = 1.0 / s(i);
    else
      sinv(i) = 0.0;
  }
  Eigen::MatrixXd jinv = (v * sinv.asDiagonal() * u.transpose());
  
  return jinv;
}



// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  RCLCPP_INFO(LOGGER, "\nSTART\n");
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (int i = 0; i < 4; ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %d: %f", i, joint_group_positions[i]);
  }

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  RCLCPP_INFO(LOGGER, "\nMY_CODE\n");

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  current_state->getJacobian(joint_model_group,
                               current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");


  Eigen::MatrixXd jinv;
  jinv = getInvJacobian(jacobian);
  RCLCPP_INFO_STREAM(LOGGER, "Jinv: \n" << jinv << "\n");

  Eigen::MatrixXd joint_torque(4,1);

  // invert direction because motor return reaction force
  joint_torque << current_state->getVariableEffort(0)*-1,
                  current_state->getVariableEffort(1)*-1,
                  current_state->getVariableEffort(2)*-1,
                  current_state->getVariableEffort(3)*-1;

  RCLCPP_INFO_STREAM(LOGGER, "joint torque: \n" << joint_torque << "\n");
  
  Eigen::MatrixXd wrench = jinv.transpose()*joint_torque;
  RCLCPP_INFO_STREAM(LOGGER, "wrench: \n" << wrench << "\n");

  Eigen::MatrixXd joint_vel(4,1);
  joint_vel << current_state->getVariableVelocity(0),
                current_state->getVariableVelocity(1),
                current_state->getVariableVelocity(2),
                current_state->getVariableVelocity(3);
  
  RCLCPP_INFO_STREAM(LOGGER, "joint vel: \n" << joint_vel << "\n");

  Eigen::MatrixXd eef_vel;
  eef_vel = jacobian * joint_vel;

  RCLCPP_INFO_STREAM(LOGGER, "eef vel: \n" << eef_vel << "\n");
  

  rclcpp::shutdown();
  return 0;
}
