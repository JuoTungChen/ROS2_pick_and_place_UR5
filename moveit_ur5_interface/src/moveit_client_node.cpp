#include <moveit_ur5_interface/moveit_pose_client.hpp>

int main( int argc, char** argv ){

  rclcpp::init(argc, argv);
  std::shared_ptr<moveit_ur5::client> client = std::make_shared<moveit_ur5::client>("client");

// Option 1: send desired pose using call(x, y ,z , q_x, q_y, q_z, q_w) 
  // auto result = client->call(0.3, -0.4, 0.5, -1, 0, 0, 0);     // can be modified

// Option 2: send desired pose using geometry_msgs
  geometry_msgs::msg::Pose pick_goal;
  pick_goal.position.x = 0.15;
  pick_goal.position.y = 0.25;
  pick_goal.position.z = 0.35;
  pick_goal.orientation.x = -1;
  pick_goal.orientation.y = 0;
  pick_goal.orientation.z = 0;
  pick_goal.orientation.w = 0;

  auto result = client->call(pick_goal); 

  rclcpp::spin_until_future_complete(client, result);
  rclcpp::shutdown();
    
  return 0;

}
