#include <pnp_actionlib/gripper_action.hpp>
// #include </home/justin/project_ws/install/robotiq_2f_msgs/include/robotiq_2f_msgs/msg/command_state.hpp>
#include <robotiq_2f_msgs/msg/command_state.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rsp{
  gripper_action::gripper_action(const std::string &name): Node(name){
      server = rclcpp_action::create_server<GripperAction>(this, name, 
		      std::bind(&gripper_action::goal_callback, this, _1, _2),
		      std::bind(&gripper_action::cancel_callback, this, _1),
		      std::bind(&gripper_action::accept_goal, this, _1));	  
  }

  rclcpp_action::GoalResponse gripper_action::goal_callback(const rclcpp_action::GoalUUID& , GripperAction::Goal::ConstSharedPtr goal){
		if(goal->gripper_goal == "open" || goal->gripper_goal == "close"){
			std::cout << "\n[gripper server: goal accepted]" <<std::endl;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		else{
			std::cout << "\n[gripper server: goal rejected]" <<std::endl;
			return rclcpp_action::GoalResponse::REJECT;
		}
  }

  rclcpp_action::CancelResponse gripper_action::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>>){
		std::cout << "\n[gripper server: action canceled]" <<std::endl;
		return rclcpp_action::CancelResponse::ACCEPT;
  }

  void gripper_action::accept_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>> goal_handle){
		std::cout << "gripper server processing" <<std::endl;
		auto gripper_feedback = std::make_shared<GripperAction::Feedback>();
		auto ros_publisher = create_publisher<robotiq_2f_msgs::msg::CommandState>("robotiq_2f_command", 10);
		auto m = robotiq_2f_msgs::msg::CommandState();
		const auto goal_pose = goal_handle -> get_goal();
		m.command = goal_pose->gripper_goal;
		ros_publisher -> publish(m);


		for(gripper_feedback->gripper_progress = 0; gripper_feedback->gripper_progress < 1; gripper_feedback->gripper_progress+=0.1){
			std::this_thread::sleep_for(std::chrono::milliseconds(gripper_step));
			goal_handle->publish_feedback(gripper_feedback);
		} 
	
		auto result = std::make_shared<GripperAction::Result>();
		result->gripper_result = "SUCCESSES";
		goal_handle->succeed(result);
  }


}
