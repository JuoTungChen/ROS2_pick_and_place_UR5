#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_ur5_msgs/srv/plan_request.hpp>

namespace moveit_ur5{
	class client: public rclcpp::Node{
		private:
			rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedPtr ros_client;

		public:
			client(const std::string& name);
			rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture call(const double &x, const double &y, const double &z, const double &q_x, const double &q_y, const double &q_z, const double &q_w);
			rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture call(const geometry_msgs::msg::Pose &target_pose);
			void callback(const rclcpp::Client<moveit_ur5_msgs::srv::PlanRequest>::SharedFuture future);
	
	};
}



