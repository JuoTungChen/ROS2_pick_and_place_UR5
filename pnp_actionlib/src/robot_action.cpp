#include <pnp_actionlib/robot_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include </home/justin/project_ws/src/moveit_ur5_interface/include/moveit_ur5_interface/moveit_pose_client.hpp>
#include <moveit_ur5_interface/moveit_pose_client.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
namespace rsp{
	
  robot_action::robot_action(const std::string &name): Node(name){
      server = rclcpp_action::create_server<RobotAction>(this, name,
		      std::bind(&robot_action::goal_callback, this, _1, _2),
		      std::bind(&robot_action::cancel_callback, this, _1),
		      std::bind(&robot_action::accept_goal, this, _1));	  

  }

  rclcpp_action::GoalResponse robot_action::goal_callback(const rclcpp_action::GoalUUID& , RobotAction::Goal::ConstSharedPtr goal){
		if(abs(goal->robot_goal.position.x) <=1){
			std::cout << "\n[robot server: goal accepted]" <<std::endl;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		else{
			std::cout << "\n[robot server: goal rejected]" <<std::endl;
			return rclcpp_action::GoalResponse::REJECT;
		}
  }

  rclcpp_action::CancelResponse robot_action::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotAction>>){
		std::cout << "\n[robot server: action canceled]" <<std::endl;
		return rclcpp_action::CancelResponse::ACCEPT;
  }
  /*

  void robot_action::accept_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotAction>> goal_handle){
		std::cout << "robot server processing" <<std::endl;
		auto robot_feedback = std::make_shared<RobotAction::Feedback>();
		for(robot_feedback->robot_progress = 0; robot_feedback->robot_progress < 1; robot_feedback->robot_progress+=0.1){
			std::this_thread::sleep_for(std::chrono::milliseconds(robot_step));
			goal_handle->publish_feedback(robot_feedback);
		} 
	
		auto result = std::make_shared<RobotAction::Result>();
		result->robot_result = "SUCCESSES";
		goal_handle->succeed(result);
  }
*/

  void robot_action::accept_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotAction>> goal_handle){
		std::cout << "robot server processing" <<std::endl;
		auto robot_feedback = std::make_shared<RobotAction::Feedback>();
		// obtain extract from goal handle
		const auto goal_pose = goal_handle -> get_goal();

		std::shared_ptr<moveit_ur5::client> client = std::make_shared<moveit_ur5::client>("robot_client");
		auto moveit_result = client->call(goal_pose->robot_goal); 
		rclcpp::executors::MultiThreadedExecutor executor;
		executor.add_node(client);
		executor.spin_until_future_complete(moveit_result);
		auto response = moveit_result.get();
		// robot_feedback->robot_progress = 0;
		// goal_handle->publish_feedback(robot_feedback);

		/*
		for(robot_feedback->robot_progress = 0; robot_feedback->robot_progress < 1; robot_feedback->robot_progress+=0.1){
			std::this_thread::sleep_for(std::chrono::milliseconds(robot_step));
			goal_handle->publish_feedback(robot_feedback);
		} 
		*/
		auto result = std::make_shared<RobotAction::Result>();
		if (response->result == "success"){
			result->robot_result = "robot action succeed";
			goal_handle->succeed(result);
		}
		else{
			result->robot_result = "robot action failed";
			goal_handle->abort(result);

		}
  }

}
