#include <rclcpp/rclcpp.hpp>
#include <moveit_ur5_msgs/srv/plan_request.hpp>

namespace moveit_ur5{

	class service : public rclcpp::Node{
		private:
			const double eef_step = 0.06;
			const double jump_threshold = 6.0;
			std::vector<geometry_msgs::msg::Pose> waypoints;

			rclcpp::Service<moveit_ur5_msgs::srv::PlanRequest>::SharedPtr ros_service;
		public:
			service(const std::string& name);
			void callback(const std::shared_ptr<moveit_ur5_msgs::srv::PlanRequest::Request> request, 
					std::shared_ptr<moveit_ur5_msgs::srv::PlanRequest::Response> response);
	};

}



