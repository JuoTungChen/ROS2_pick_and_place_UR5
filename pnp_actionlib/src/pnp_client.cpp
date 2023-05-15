#include <pnp_actionlib/pnp_action.hpp>
#include <pnp_actionlib/robot_action.hpp>
#include <pnp_actionlib/gripper_action.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    
    std::shared_ptr<rsp::pnp_client> pnpc = std::make_shared<rsp::pnp_client>("pnp");

    geometry_msgs::msg::Pose pick_goal;
    pick_goal.position.x = 0.3;
    pick_goal.position.y = 0.4;
    pick_goal.position.z = 0.3;
    pick_goal.orientation.x = -1;
    pick_goal.orientation.y = 0;
    pick_goal.orientation.z = 0;
    pick_goal.orientation.w = 0;

    geometry_msgs::msg::Pose place_goal;
    place_goal.position.x = -0.25;
    place_goal.position.y = 0.4;
    place_goal.position.z = 0.3;
    place_goal.orientation.x = -1;
    place_goal.orientation.y = 0;
    place_goal.orientation.z = 0;
    place_goal.orientation.w = 0;

    std::shared_future<rclcpp_action::ClientGoalHandle<PNPAction>::SharedPtr> client_result = pnpc->pnp_call(pick_goal, place_goal);

    // rclcpp::spin_until_future_complete(pnpc, client_result);
    rclcpp::spin(pnpc);

    rclcpp::shutdown();
    return 0;
}
