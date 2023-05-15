#include <pnp_actionlib/pnp_action.hpp>
#include <pnp_actionlib/robot_action.hpp>
#include <pnp_actionlib/gripper_action.hpp>


int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    std::shared_ptr<rsp::robot_action> rs = std::make_shared<rsp::robot_action>("robot");
    std::shared_ptr<rsp::gripper_action> gs = std::make_shared<rsp::gripper_action>("gripper");
    std::shared_ptr<rsp::pnp_action> pnps = std::make_shared<rsp::pnp_action>("pnp");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rs);
    executor.add_node(gs);
    executor.add_node(pnps);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
