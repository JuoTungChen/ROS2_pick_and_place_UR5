cmake_minimum_required(VERSION 3.8)
project(pnp_actionlib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pnp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotiq_2f_msgs REQUIRED)
find_package(moveit_ur5_interface REQUIRED)
find_package(moveit_ur5_msgs REQUIRED)

include_directories(include)

add_library(pnp SHARED src/robot_action.cpp src/gripper_action.cpp src/pnp_action.cpp)
ament_target_dependencies(pnp robotiq_2f_msgs moveit moveit_ros_planning_interface moveit_ur5_interface rclcpp rclcpp_action pnp_msgs moveit_ur5_msgs)

add_executable(server src/pnp_server.cpp)
target_link_libraries(server pnp)
ament_target_dependencies(server robotiq_2f_msgs moveit moveit_ros_planning_interface moveit_ur5_interface rclcpp rclcpp_action pnp_msgs moveit_ur5_msgs)

add_executable(client src/pnp_client.cpp)
target_link_libraries(client pnp)
ament_target_dependencies(client rclcpp rclcpp_action robotiq_2f_msgs pnp_msgs moveit moveit_ros_planning_interface moveit_ur5_interface moveit_ur5_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS pnp LIBRARY DESTINATION lib)
install(TARGETS server client DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(pnp)


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
