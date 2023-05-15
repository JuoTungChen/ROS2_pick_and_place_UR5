
# ROS2 pick and place for UR5 using Moveit2

This repo provides ROS2 packages for initiating a pick-and-place service for the UR5 Robotic Manipulator combining a Robotiq85 2 DoF gripper.

It is part of the following project: [__`AR pick and place with UR5`__](https://github.com/vishnukolal/rsp_project)

</br>

### Prerequisites

The following Linux version as well as the ROS distribution is required:

- Ubuntu 20.04 / ROS 2 Galactic Geochelone

</br>

## Packages
1. __moveit_ur5_interface__
2. __moveit_ur5_msgs__
3. __pnp_actionlib__
4. __pnp_msgs__
5. __robotiq_description__
6. __ur5_gripper_description__
7. __ur5_gripper_moveit_config__

</br>

## Moveit UR5 Interface

This package provides a service/client interface between the official Universal Robots ROS2 Driver and the Moveit2 API. The package takes position commands in the form of geometry_msgs/msg/Pose via ROS2 service and generates a trajectory. It then uses the UR5 ROS2 driver to control the robot and follow the trajectory.

</br>

### Launch Files

The package provides the following launch file:

1. __`ros2 launch moveit_ur5_interface moveit_ur5_interface_sim.launch`__  
This launch file is used when you don't have the actual UR5 and want to see it in simulation. It starts the UR5 driver with fake hardware controller and the Moveit2 server. It launches the UR5 Moveit configuration with the gripper attached to the end effector and opens Rviz. You can call services or run a client node that publishes the target pose to move the robot to anywhere you want.
 
2. __`ros2 launch moveit_ur5_interface moveit_ur5_interface.launch`__  
This launch file is used when you want to actually control a real UR5. 
It starts up the UR5 driver and the Moveit2 server with custom settings. It launches the UR5 Moveit configuration with the gripper attached to the end effector and opens Rviz.  
To connect to a real UR5, you can follow this instruction: [__Setting up a UR robot for ur_robot_driver__](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/galactic/ur_robot_driver/doc/installation/robot_setup.rst)  
After finishing setting up the robot, you can run the launch file, and press the play button on the pendant.

</br>

### Executables

The package provides the following executables:

1. __moveit_server_node.cpp__  
    ```
    ros2 run moveit_ur5_interface moveit_srv_node
    ```
    This node initiates a server that communicates with the move group node when receiving a planning request.


2. __moveit_client_node.cpp__  
    ```
    ros2 run moveit_ur5_interface moveit_cli_node
    ```  
    This is an example client node that sends a planning request to the moveit_server_node.cpp. It constructs and sends a request to the server by calling the following function (overloaded with two variants):

    ```
    call(const double &x, const double &y, const double &z, const double &q_x, const double &q_y, const double &q_z, const double &q_w);

    call(const geometry_msgs::msg::Pose &target_pose);
    ```

3. __moveit_interface.cpp__  
    ```
    ros2 run moveit_ur5_interface moveit_interface
    ```  
    This is an simple example of using the Moveit C++ API to control the UR5. It uses the move group to plan and execute a path for the UR5. It does the same thing as moveit_server but without the ability to initiate a planning request from the client.

</br>


### Usage

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
