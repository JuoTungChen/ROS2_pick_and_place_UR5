#include <pnp_actionlib/pnp_action.hpp>
#include <pnp_actionlib/robot_action.hpp>
#include <pnp_actionlib/gripper_action.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rsp{
	
  pnp_action::pnp_action(const std::string &name):Node(name){
      server = rclcpp_action::create_server<PNPAction>(this, name, 
		      std::bind(&pnp_action::goal_callback, this, _1, _2),
		      std::bind(&pnp_action::cancel_callback, this, _1),
		      std::bind(&pnp_action::accept_goal, this, _1));	  
	  }

  rclcpp_action::GoalResponse pnp_action::goal_callback(const rclcpp_action::GoalUUID& , PNPAction::Goal::ConstSharedPtr goal){
		std::cout << "pnp_server: goal callback" <<std::endl;
		double pick_reach = std::sqrt(std::pow(goal->pick_goal.position.x, 2) + std::pow(goal->pick_goal.position.y, 2) + std::pow(goal->pick_goal.position.z, 2));
		double place_reach = std::sqrt(std::pow(goal->place_goal.position.x, 2) + std::pow(goal->place_goal.position.y, 2) + std::pow(goal->place_goal.position.z, 2));
		if(pick_reach <= UR5_MAX_REACH && place_reach <= UR5_MAX_REACH && goal->place_goal.position.z >= 0 && goal->pick_goal.position.z >= 0){
			std::cout << "[pnp server: goal accepted]" <<std::endl;
			callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
			RobotClient = rclcpp_action::create_client<RobotAction>(this, "robot", callback_group);
			RobotClient->wait_for_action_server();
			GripperClient = rclcpp_action::create_client<GripperAction>(this, "gripper", callback_group);
			GripperClient->wait_for_action_server();
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}
		else{
			std::cout << "[pnp server: goal rejected]" <<std::endl;
			return rclcpp_action::GoalResponse::REJECT;
		}
  }

  rclcpp_action::CancelResponse pnp_action::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PNPAction>>){
		std::cout << "pnp server: action canceled" <<std::endl;
		return rclcpp_action::CancelResponse::ACCEPT;
  }

  void pnp_action::accept_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PNPAction>> goal_handle){
		std::cout << "[pnp server processing]" <<std::endl;
		const auto goals = goal_handle -> get_goal();
        geometry_msgs::msg::Pose inter_pick_pose = goals->pick_goal;
        geometry_msgs::msg::Pose inter_place_pose = goals->place_goal;
        inter_pick_pose.position.z += 0.2;
        inter_place_pose.position.z += 0.2;
		auto pnp_feedback = std::make_shared<PNPAction::Feedback>();

		double x1 = goals->pick_goal.position.x; 
		double y1 = goals->pick_goal.position.y; 
		double x2 = goals->place_goal.position.x; 
		double y2 = goals->place_goal.position.y; 

		double angle1 = std::atan2(y1, x1);
		double angle2 = std::atan2(y2, x2);
		double angle = angle2 - angle1;
		
		// Adjust the angle to be between -pi and pi
		if (angle > M_PI) {
			angle -= 2 * M_PI;
		} else if (angle < -M_PI) {
			angle += 2 * M_PI;
		}
		double angle_deg = abs(angle * (180.0 / M_PI));
		

	/*---- MOVE TO PICK ----*/
		pnp_feedback->pnp_message = "moving to pick the object";
		goal_handle->publish_feedback(pnp_feedback);
    	robot_call(inter_pick_pose);
		robot_complete = false;
		while(!robot_complete){
			std::this_thread::sleep_for(100ms);
		}
    	robot_call(goals->pick_goal);
		robot_complete = false;
		while(!robot_complete){
			std::this_thread::sleep_for(100ms);
		}
	/*---- CLOSE GRIPPER ----*/
		pnp_feedback->pnp_message = "closing the gripper";
		goal_handle->publish_feedback(pnp_feedback);
		gripper_call("close");
		gripper_complete = false;
		while(!gripper_complete){
			std::this_thread::sleep_for(100ms);
		}

	// generate a waypoint if the angle > 120
		// if ( angle_deg > 120 ){
			
		// }


	/*---- MOVE TO PLACE ----*/
		pnp_feedback->pnp_message = "moving to place the object";
		goal_handle->publish_feedback(pnp_feedback);
		robot_call(inter_place_pose);
		robot_complete = false;
		while(!robot_complete){
			std::this_thread::sleep_for(100ms);
		}
		robot_call(goals->place_goal);
		robot_complete = false;
		while(!robot_complete){
			std::this_thread::sleep_for(100ms);
		}

	/*---- OPEN GRIPPER ----*/
		pnp_feedback->pnp_message = "opening the gripper";
		goal_handle->publish_feedback(pnp_feedback);
		gripper_call("open");
		gripper_complete = false;
		while(!gripper_complete){
			std::this_thread::sleep_for(100ms);
		}
		
	/*---- SHOW RESULT ----*/
		auto result = std::make_shared<PNPAction::Result>();
		result->pnp_result = "SUCCESSES";
		goal_handle->succeed(result);
  }

	/*---- PNP client ----*/
	pnp_client::pnp_client(const std::string &name):
		Node(name){
			pc = rclcpp_action::create_client<PNPAction>(this, name);
			pc -> wait_for_action_server();
		}

	std::shared_future<rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr> pnp_client::pnp_call(const geometry_msgs::msg::Pose &pick_destination, const geometry_msgs::msg::Pose &place_destination){
		
		PNPAction::Goal goal;
		goal.pick_goal = pick_destination;
		goal.place_goal = place_destination;

		std::cout << "[sending goal]" << std::endl;
		std::cout << "[pick position:] x = " << goal.pick_goal.position.x << ", y = " << goal.pick_goal.position.y << ", z = " << goal.pick_goal.position.z << std::endl;
		std::cout << "[place position:] x = " << goal.place_goal.position.x << ", y = " << goal.place_goal.position.y << ", z = " << goal.place_goal.position.z << std::endl;

		rclcpp_action::Client<PNPAction>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&pnp_client::response_callback, this, _1);
		options.feedback_callback = std::bind(&pnp_client::feedback_callback, this, _1, _2);
		options.result_callback = std::bind(&pnp_client::result_callback, this, _1);
		auto future_result = pc -> async_send_goal(goal, options);
		// auto response = future_result.get();
		return future_result;
	}

	void pnp_client::response_callback(rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr)
	{
		std::cout << "[pnp client response:] " << std::endl;
	}
	void pnp_client::feedback_callback(rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr , const std::shared_ptr<const PNPAction::Feedback> feedback)
	{
		std::cout << "[pnp client feedback:] " << feedback->pnp_message <<std::endl;
	}
	void pnp_client::result_callback(const rclcpp_action::ClientGoalHandle<PNPAction>::WrappedResult& result)
	{
		std::cout << "[pnp client result:] " << result.result->pnp_result <<std::endl;
	}


// ---------------- Robot Client -------------------

	void pnp_action::robot_call(const geometry_msgs::msg::Pose &goal_pose){

		RobotAction::Goal goal;
		goal.robot_goal = goal_pose;

		rclcpp_action::Client<RobotAction>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&pnp_action::robot_response_callback, this, _1);
		options.feedback_callback = std::bind(&pnp_action::robot_feedback_callback, this, _1, _2);
		options.result_callback = std::bind(&pnp_action::robot_result_callback, this, _1);
		RobotClient -> async_send_goal(goal, options);
	}

	void pnp_action::robot_response_callback(rclcpp_action::ClientGoalHandle<RobotAction>::SharedPtr )
	{
		std::cout << "[robot client response:]" << std::endl;
	}
	void pnp_action::robot_feedback_callback(rclcpp_action::ClientGoalHandle<RobotAction>::SharedPtr , const std::shared_ptr<const RobotAction::Feedback> feedback)
	{
		std::cout << "[robot feedback:] " << feedback->robot_progress <<std::endl;
	}
	void pnp_action::robot_result_callback(const rclcpp_action::ClientGoalHandle<RobotAction>::WrappedResult& result)
	{
		std::cout << "[robot result:] " << result.result->robot_result <<std::endl;
		robot_complete = true;
	}


// ---------------- Gripper Client -------------------

	void pnp_action::gripper_call(const std::string& command){
		GripperAction::Goal goal;
		goal.gripper_goal = command;
		rclcpp_action::Client<GripperAction>::SendGoalOptions options;
		options.goal_response_callback = std::bind(&pnp_action::gripper_response_callback, this, _1);
		options.feedback_callback = std::bind(&pnp_action::gripper_feedback_callback, this, _1, _2);
		options.result_callback = std::bind(&pnp_action::gripper_result_callback, this, _1);
		GripperClient -> async_send_goal(goal, options);
	}

	void pnp_action::gripper_response_callback(rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr )
	{
		std::cout << "[gripper client response:]" << std::endl;
	}
	void pnp_action::gripper_feedback_callback(rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr, const std::shared_ptr<const GripperAction::Feedback> feedback)
	{
		std::cout << "[gripper feedback:] " << feedback->gripper_progress <<std::endl;
	}
	void pnp_action::gripper_result_callback(const rclcpp_action::ClientGoalHandle<GripperAction>::WrappedResult& result)
	{
		std::cout << "[gripper result:] " << result.result->gripper_result <<std::endl;
		gripper_complete = true;

	}
}
