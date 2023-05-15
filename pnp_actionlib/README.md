# PNP actionlib

The package uses ROS actionlib to implement a Pick-and-Place (PNP) action server that accepts a goal to pick up an object at a specified location and place it at another specified location, both in the form of geometry_msgs/msg/Pose.

</br>


## Launch Files

The package provides the following launch file:

1. __ur5_pnp_actionlib.launch__

This launch file starts the UR5 driver and the Moveit2 server with custom settings. It launches the UR5 Moveit configuration with the UR type selected as UR5e and opens Rviz. The fake trajectory following controller for UR5e with ROS2 control is also launched. Finally, it launches the pnp_server which will initiate a PNP service when called by a client.

</br>

## Executables

The package provides the following executables:

1. __pnp_server.cpp__

This node initiates a action server that initiate a PNP service when receiving a planning request.


2. __pnp_client.cpp__

This is an example client node that sends a pnp request to the pnp_server.cpp. 

</br>


## Usage

1. To send a planning request from the command line, use the following commands:

```
ros2 launch pnp_actionlib ur5_pnp_actionlib.launch 

ros2 action send_goal /pnp pnp_msgs/action/PickAndPlace "{pick_goal:{position: {x: -0.3, y: -0.4, z: 0.1}, orientation: {x: -1.0, y: 0.0, z: 0.0, w: 0.0}}, place_goal:{position: {x: -0.5, y: -0.2, z: 0.1}, orientation: {x: -1.0, y: 0.0, z: 0.0, w: 0.0}}}"

```

2. To send a plan request from the client node, modify the target within the pnp_client.cpp file and use the following commands:

```
ros2 launch pnp_actionlib ur5_pnp_actionlib.launch 

ros2 run pnp_actionlib client
```


