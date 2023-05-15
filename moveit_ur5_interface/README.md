# Moveit UR5 Interface


This package provides a service/client interface between the official Universal Robots ROS2 Driver and the Moveit2 API. The package takes position commands in the form of geometry_msgs/msg/Pose via ROS2 service and generates a trajectory. It then uses the UR5 ROS2 driver to control the robot and follow the trajectory.

</br>


## Launch Files

The package provides the following launch file:

1. __moveit_ur5_interface.launch__

This launch file starts the UR5 driver and the Moveit2 server with custom settings. It launches the UR5 Moveit configuration with the UR type selected as UR5e and opens Rviz. The fake controller for UR5e with ROS2 control is also launched.
 

</br>

## Executables

The package provides the following executables:

1. __moveit_server_node.cpp__

This node initiates a server that communicates with the move group node when receiving a planning request.


2. __moveit_client_node.cpp__

This is an example client node that sends a planning request to the moveit_server_node.cpp. It constructs and sends a request to the server by calling the following function (overloaded with two variants):

```
call(const double &x, const double &y, const double &z, const double &q_x, const double &q_y, const double &q_z, const double &q_w);

call(const geometry_msgs::msg::Pose &target_pose);
```

3. __moveit_interface.cpp__

This is an example of using the Moveit C++ API to control the UR5. It uses the move group to plan and execute a path for the UR5. It does the same thing as moveit_server but without the ability to initiate a planning request from the client.

</br>


## Usage

1. To send a planning request from the command line, use the following commands:

```
ros2 launch moveit_ur5_interface moveit_ur5_interface.launch 

ros2 service call /moveit_ur5/target_pose moveit_ur5_msgs/srv/PlanRequest "{target:{position: {x: -0.3, y: -0.4, z: 0.6}, orientation: {x: -1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

2. To send a plan request from the client node, modify the target within the moveit_client_node.cpp file and use the following commands:

```
ros2 launch moveit_ur5_interface moveit_ur5_interface.launch 

ros2 run moveit_ur5_interface moveit_cli_node
```


