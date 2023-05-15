cmake_minimum_required(VERSION 3.8)
project(moveit_ur5_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur_description REQUIRED)
find_package(ur_bringup REQUIRED)
find_package(moveit_ur5_msgs REQUIRED)
find_package(ur5_gripper_description REQUIRED)
find_package(ur5_gripper_moveit_config REQUIRED)

include_directories(include)
add_library(srvcli SHARED src/moveit_pose_server.cpp src/moveit_pose_client.cpp)
ament_target_dependencies(srvcli rclcpp moveit_ur5_msgs moveit moveit_ros_planning_interface ur5_gripper_description)

add_executable(moveit_interface src/moveit_interface.cpp)
ament_target_dependencies(moveit_interface rclcpp moveit_ros_planning_interface moveit ur_description ur5_gripper_description)

add_executable(moveit_cli_node src/moveit_client_node.cpp)
target_link_libraries(moveit_cli_node srvcli)
ament_target_dependencies(moveit_cli_node rclcpp moveit_ur5_msgs moveit_ros_planning_interface moveit ur_description ur5_gripper_description)

add_executable(moveit_srv_node src/moveit_server_node.cpp)
target_link_libraries(moveit_srv_node srvcli)
ament_target_dependencies(moveit_srv_node rclcpp moveit_ur5_msgs moveit_ros_planning_interface moveit ur_description ur5_gripper_description)


install(DIRECTORY include/ DESTINATION include)
install(TARGETS srvcli LIBRARY DESTINATION lib)

install(TARGETS moveit_interface moveit_cli_node moveit_srv_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(srvcli)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
