#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pnp_msgs/action/gripper.hpp>
// #include </home/justin/project_ws/install/robotiq_2f_msgs/include/robotiq_2f_msgs/msg/command_state.hpp>

using GripperAction = pnp_msgs::action::Gripper;

namespace rsp{

  class gripper_action : public rclcpp::Node{
	public:
	   gripper_action(const std::string &name);
	   rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID&, GripperAction::Goal::ConstSharedPtr goal);
	   rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>>);
	   void accept_goal(std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>> goal_handle);

   	private: 
	   const int gripper_step = 200;
	   rclcpp_action::Server<GripperAction>::SharedPtr server;

  };

}
