#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pnp_msgs/action/robot.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using RobotAction = pnp_msgs::action::Robot;
namespace rsp{

  class robot_action : public rclcpp::Node{
	public:
	    robot_action(const std::string &name);
	    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID&, RobotAction::Goal::ConstSharedPtr goal);
	    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotAction>>);
	    void accept_goal(std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotAction>> goal_handle);

   	private: 
	   rclcpp_action::Server<RobotAction>::SharedPtr server;
	   const int robot_step = 500;
  };

}
