#include <moveit_ur5_msgs/srv/plan_request.hpp>
#include <moveit_ur5_interface/moveit_pose_server.hpp>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

    std::shared_ptr<moveit_ur5::service> moveit_server = std::make_shared<moveit_ur5::service>("moveit_interface_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(moveit_server);
    executor.spin();
    rclcpp::shutdown();

	return 0;
}



