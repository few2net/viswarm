#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <chrono>
#include <thread>

// using namespace std::chrono_literals;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
static const rclcpp::Logger LOGGER = rclcpp::get_logger("wrench_publisher");


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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // auto node = rclcpp::Node::make_shared("wrench_publisher");
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("wrench_publisher", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  RCLCPP_INFO_STREAM(LOGGER, "ready!: \n");
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  auto publisher_ = move_group_node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench", 10);

  // auto timer_ = move_group_node->create_wall_timer(500ms, std::bind(timer_callback, move_group_node));

  rclcpp::Rate loop_rate(100); // 10 Hz
  while (rclcpp::ok()) {
    // Publish messages here
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;

    current_state->getJacobian(joint_model_group,
                              current_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                              reference_point_position, jacobian);
    
    // RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

    // Eigen::MatrixXd jinv;
    // jinv = getInvJacobian(jacobian);

    Eigen::MatrixXd reduced_jac(4,4);
    reduced_jac << jacobian(0,0), jacobian(0,1), jacobian(0,2), jacobian(0,3),
                  jacobian(2,0), jacobian(2,1), jacobian(2,2), jacobian(2,3),
                  jacobian(4,0), jacobian(4,1), jacobian(4,2), jacobian(4,3),
                  jacobian(5,0), jacobian(5,1), jacobian(5,2), jacobian(5,3);

    Eigen::MatrixXd joint_torque(4,1);

    // invert direction because motor return reaction force
    joint_torque << current_state->getVariableEffort(0)*-1,
                    current_state->getVariableEffort(1)*-1,
                    current_state->getVariableEffort(2)*-1,
                    current_state->getVariableEffort(3)*-1;

    // Eigen::MatrixXd wrench = jinv.transpose()*joint_torque;
    Eigen::MatrixXd reduced_wrench = reduced_jac.transpose().inverse()*joint_torque;

    // RCLCPP_INFO_STREAM(LOGGER, "torque: \n" << joint_torque << "\n");
    // RCLCPP_INFO_STREAM(LOGGER, "wrench: \n" << reduced_wrench << "\n");

    auto message = geometry_msgs::msg::WrenchStamped();
    rclcpp::Time current_time = move_group_node->get_clock()->now();
    message.header.stamp = current_time;

    message.wrench.force.x = reduced_wrench(0,0);
    message.wrench.force.y = 0.0;
    message.wrench.force.z = reduced_wrench(1,0);
    message.wrench.torque.x = 0.0;
    message.wrench.torque.y = reduced_wrench(2,0);
    message.wrench.torque.z = reduced_wrench(3,0);
    publisher_->publish(message);

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}