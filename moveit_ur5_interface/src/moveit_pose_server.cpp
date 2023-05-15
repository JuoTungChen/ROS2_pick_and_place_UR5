
#include <moveit_ur5_interface/moveit_pose_server.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <time.h>       

using namespace std::chrono_literals;

namespace moveit_ur5{

	service::service(const std::string& name):
		Node(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
            ros_service = create_service<moveit_ur5_msgs::srv::PlanRequest>("moveit_ur5/target_pose", 
                    std::bind(&service::callback, this, std::placeholders::_1, std::placeholders::_2));
		}

	void service::callback(const std::shared_ptr<moveit_ur5_msgs::srv::PlanRequest::Request> request,
		std::shared_ptr<moveit_ur5_msgs::srv::PlanRequest::Response> response){
		std::cout << "service: received" << std::endl;
        
        // Create a ROS logger
        auto const logger = rclcpp::get_logger("hello_moveit");

        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(shared_from_this(), "ur_manipulator"); //recycle the server node as node for MoveGroupInterface
        move_group_interface.setEndEffector("ee_link");        
        
        // Cartesian Path Planning
        waypoints.resize(1);
        waypoints[0] = (request->target);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double planning_result = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        // Execute the plan
        if(planning_result) {
            auto const execution_result = move_group_interface.execute(trajectory);     // execute the trajectory
            if (execution_result){
                response->result = "success";    //return success to the client if the execution is successful
            }
            else{
                response->result = "execution failed!";
                RCLCPP_ERROR(logger, "Execution failed!");
                }
        } 
        else {
            response->result = "planning failed!";
            RCLCPP_ERROR(logger, "Planning failed!");
        }

		std::cout<< "service: done" << std::endl;
	}

}



