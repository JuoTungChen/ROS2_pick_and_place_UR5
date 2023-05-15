#include <moveit_ur5_interface/moveit_pose_client.hpp>

namespace moveit_ur5{

	client::client(const std::string& name):
		Node(name){
			ros_client = create_client<moveit_ur5_msgs::srv::PlanRequest>( "moveit_ur5/target_pose");

			std::cout << "client: wait for service" << std::endl;
			ros_client->wait_for_service();
			std::cout << "client: ready" << std::endl;
		}

	rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture client::call(const double &x, const double &y, const double &z, const double &q_x, const double &q_y, const double &q_z, const double &q_w){
		auto request = std::make_shared<moveit_ur5_msgs::srv::PlanRequest::Request>();
		geometry_msgs::msg::Pose target_pose;
		target_pose.position.x = x; 
		target_pose.position.y = y; 
		target_pose.position.z = z; 
		target_pose.orientation.x = q_x; 
		target_pose.orientation.y = q_y; 
		target_pose.orientation.z = q_z; 
		target_pose.orientation.w = q_w; 

		request->target = target_pose;
		std::cout << "client: sending" << std::endl;
		auto result = ros_client->async_send_request(request, std::bind(&client::callback, this, std::placeholders::_1 ));
		std::cout << "client: sent" << std::endl;
		return result;
	}

	// overload
	rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture client::call(const geometry_msgs::msg::Pose &target_pose){
		auto request = std::make_shared<moveit_ur5_msgs::srv::PlanRequest::Request>();
		request->target = target_pose;
		std::cout << "client: sending" << std::endl;
		auto result = ros_client->async_send_request(request, std::bind(&client::callback, this, std::placeholders::_1 ));
		std::cout << "client: sent" << std::endl;
		return result;
	}
	
	void client::callback(const rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture future){
		std::cout << "client: callback" << std::endl;
		auto response = future.get();
		std::cout << "moveit result = " << response->result << std::endl;
	}

}



