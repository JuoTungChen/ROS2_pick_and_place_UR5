#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pnp_msgs/action/robot.hpp>
#include <pnp_msgs/action/gripper.hpp>
#include <pnp_msgs/action/pick_and_place.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using PNPAction = pnp_msgs::action::PickAndPlace;
using RobotAction = pnp_msgs::action::Robot;
using GripperAction = pnp_msgs::action::Gripper;


namespace rsp{

  class pnp_action : public rclcpp::Node{
	public:
		pnp_action(const std::string &name);
		rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID&, PNPAction::Goal::ConstSharedPtr goal);
		rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PNPAction>>);
		void accept_goal(std::shared_ptr<rclcpp_action::ServerGoalHandle<PNPAction>> goal_handle);
		void robot_call(const geometry_msgs::msg::Pose &goal_pose);		
		void robot_response_callback(rclcpp_action::ClientGoalHandle<RobotAction>::SharedPtr handle);
		void robot_feedback_callback(rclcpp_action::ClientGoalHandle<RobotAction>::SharedPtr handle, const std::shared_ptr<const RobotAction::Feedback> feedback);
		void robot_result_callback(const rclcpp_action::ClientGoalHandle<RobotAction>::WrappedResult& result);
		void gripper_call(const std::string& command);
		void gripper_response_callback(rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr handle);
		void gripper_feedback_callback(rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr handle, const std::shared_ptr<const GripperAction::Feedback> feedback);
		void gripper_result_callback(const rclcpp_action::ClientGoalHandle<GripperAction>::WrappedResult& result);

   	private: 
	   	bool robot_complete = false;		
		bool gripper_complete = false;
		rclcpp::CallbackGroup::SharedPtr callback_group;
		rclcpp_action::Server<PNPAction>::SharedPtr server;
		rclcpp_action::Client<RobotAction>::SharedPtr RobotClient;
		rclcpp_action::Client<GripperAction>::SharedPtr GripperClient;
		const double UR5_MAX_REACH = 1.0; // Maximum reach of UR5 in meters

  };

  class pnp_client: public rclcpp::Node{
	private:
		rclcpp_action::Client<PNPAction>::SharedPtr pc;
	public:
		pnp_client(const std::string& name);
		std::shared_future<rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr> pnp_call(const geometry_msgs::msg::Pose &pick_destination, const geometry_msgs::msg::Pose &place_destination);
		void response_callback(rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr handle);
		void feedback_callback(rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr handle, const std::shared_ptr<const PNPAction::Feedback> feedback);
		void result_callback(const rclcpp_action::ClientGoalHandle<PNPAction>::WrappedResult& result);
  };

}
