#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp" 

#include <chrono>
#include <thread>

// using namespace std::chrono_literals;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik_solver_node");
//rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

/*

void command_cb(const geometry_msgs::msg::Vector3 & msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

kinematic_state->setToRandomPositions(joint_model_group);
const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

//Print end-effector pose. Remember that this is in the model frame
RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");


moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Did not find IK solution");
  }


}
*/

// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalPublisher
{
  public:
    MinimalPublisher()
    {
      rclcpp::NodeOptions node_options;
      node_options.automatically_declare_parameters_from_overrides(true);
      node_handle = rclcpp::Node::make_shared("ik_solver_node", node_options);

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node_handle);
      std::thread t1([&executor]() { executor.spin(); });


      move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_handle, PLANNING_GROUP);
      
      // const moveit::core::JointModelGroup* joint_model_group =
      //   move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      // moveit::core::RobotStatePtr current_state = move_group->getCurrentState(10);
      // //
      // // Next get the current set of joint values for the group.
      // std::vector<double> joint_group_positions;
      // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      // for (int i = 0; i < 4; ++i)
      // {
      //   RCLCPP_INFO(LOGGER, "Joint %d: %f", i, joint_group_positions[i]);
      // }



      // RCLCPP_INFO_STREAM(LOGGER, "ready1\n");
      // joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
     

      eef_sub = node_handle->create_subscription<geometry_msgs::msg::Vector3>(
        "eef_command", 10, std::bind(&MinimalPublisher::eef_callback, this, _1));
      
      RCLCPP_INFO_STREAM(LOGGER, "okkkkkkkkk!: \n");
      // RCLCPP_INFO_STREAM(LOGGER, rclcpp::Clock(RCL_ROS_TIME).now());
      // rclcpp::spin(node_handle);
      t1.join();
    }

  private:
    std::shared_ptr<rclcpp::Node> node_handle;
    const std::string PLANNING_GROUP = "arm";
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    moveit::core::JointModelGroup* joint_model_group;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr eef_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub;

    void eef_callback(const geometry_msgs::msg::Vector3 & msg) const
    {
      RCLCPP_INFO_STREAM(LOGGER, "ready!: \n");
      RCLCPP_INFO_STREAM(LOGGER, msg.x);
      auto test_joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
      RCLCPP_INFO_STREAM(LOGGER, "ready22: \n");
      const Eigen::Isometry3d& end_effector_state = current_state->getGlobalLinkTransform("end_effector_link");

      double timeout = 0.1;
      bool found_ik = current_state->setFromIK(test_joint_model_group, end_effector_state, timeout);
      RCLCPP_DEBUG_STREAM(LOGGER, found_ik);
      // RCLCPP_DEBUG_STREAM(LOGGER, "test1: ");
      // RCLCPP_DEBUG_STREAM(LOGGER, "test: " << msg.x);
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  MinimalPublisher();
  rclcpp::shutdown();
  return 0;
}





/*




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto ik_node = rclcpp::Node::make_shared("ik_solver_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ik_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(ik_node, PLANNING_GROUP);

  RCLCPP_INFO_STREAM(LOGGER, "ready!: \n");
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  auto joint_pub = ik_node->create_publisher<trajectory_msgs::msg::JointTrajectory>("arm_controller/joint_trajectory", 10);
  auto command_sub = ik_node->create_subscription<geometry_msgs::msg::Vector3>(
      "eef_command", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));


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
    
    Eigen::MatrixXd jinv;
    jinv = getInvJacobian(jacobian);

    Eigen::MatrixXd joint_torque(4,1);

    // invert direction because motor return reaction force
    joint_torque << current_state->getVariableEffort(0)*-1,
                    current_state->getVariableEffort(1)*-1,
                    current_state->getVariableEffort(2)*-1,
                    current_state->getVariableEffort(3)*-1;

    Eigen::MatrixXd wrench = jinv.transpose()*joint_torque;

    auto message = geometry_msgs::msg::Wrench();
    message.force.x = wrench(0,0);
    message.force.y = wrench(1,0);
    message.force.z = wrench(2,0);
    message.torque.x = wrench(3,0);
    message.torque.y = wrench(4,0);
    message.torque.z = wrench(5,0);
    publisher_->publish(message);

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

/*

kinematic_state->setToRandomPositions(joint_model_group);
const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

//Print end-effector pose. Remember that this is in the model frame
RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

double timeout = 0.1;
bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

if (found_ik)
{
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
}
else
{
  RCLCPP_INFO(LOGGER, "Did not find IK solution");
}

*/
