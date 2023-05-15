
# ROS2 pick and place for UR5 using Moveit2

This repo provides ROS2 packages for initiating a pick-and-place service for the UR5 Robotic Manipulator combining a Robotiq85 2 DoF gripper.

__It is part of the following project:__  [__`AR pick and place with UR5`__](https://github.com/vishnukolal/rsp_project)


## Prerequisites

The following Linux version as well as the ROS distribution is required:  
- Ubuntu 20.04 / ROS 2 Galactic Geochelone

</br>

## Packages in the Repository:
1. __`moveit_ur5_msgs`__ - custom message package containing one srv message for moveit_ur5_interface.
2. __`moveit_ur5_interface`__ - a service/client interface to control the UR5 robot using moveit2.
3. __`pnp_msgs`__ - custom message package containing three action messages for pnp action.
4. __`pnp_actionlib`__ - implementation of a pnp action service for the UR5 robot.
5. __`robotiq_description`__ - description files for the robotiq gripper: meshes, URDF/XACRO files, etc.
6. __`ur5_gripper_description`__ - description files for the UR5 combined with the gripper: meshes, URDF/XACRO files, etc.
7. __`ur5_gripper_moveit_config`__ - MoveIt configuration for UR5 with the gripper.

</br>

## Installation
- First you'll need to clone this repo into the src folder of your workspace
    ```
    cd <COLCON_WS>/src
    git clone https://github.com/JuoTungChen/ROS2_pick_and_place_UR5.git
    ```

- To control the UR5, the official [UR5 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic) needs to be installed.  
You can install it by running the following command:  
    ```
    vcs import src < galactic.repos
    ```

- To control the gripper, the official [gripper driver](https://github.com/sequenceplanner/robotiq_2f.git) needs to be installed.  
You can follow this link to setup the driver for the gripper: [gripper driver installation guide](https://github.com/vishnukolal/rsp_project/blob/main/src/README_gripper_instruction.md)

</br>


## Moveit UR5 msgs

This package provides a custom service type to request trajectory planning from Moveit2. 

The message moveit_ur5_msgs/srv/PlanRequest has the following format: 

```
geometry_msgs/Pose target 

--- 

string result 
```

</br>

## Moveit UR5 Interface

This package provides a service/client interface between the official Universal Robots ROS2 Driver and the Moveit2 API. The package takes position commands through the custom service `moveit_ur5_msgs` described above and generates a trajectory for the robot using Moveit2. It then uses the UR5 ROS2 driver to control the robot and follow the trajectory.

### Launch Files

The package provides the following launch files:

1. __`ros2 launch moveit_ur5_interface moveit_ur5_interface_sim.launch`__  
This launch file is used when you don't have the actual UR5 and want to see it in simulation. It starts the UR5 driver with fake hardware controller and the Moveit2 server. It launches the UR5 Moveit configuration with the gripper attached to the end effector and opens Rviz. You can call services or run a client node that publishes the target pose to move the robot to anywhere you want.
 
2. __`ros2 launch moveit_ur5_interface moveit_ur5_interface.launch`__  
This launch file is used when you want to actually control a real UR5. 
It starts up the UR5 driver and the Moveit2 server with custom settings. It launches the UR5 Moveit configuration with the gripper attached to the end effector and opens Rviz.  
To connect to a real UR5, you can follow this instruction: [__Setting up a UR robot for ur_robot_driver__](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/galactic/ur_robot_driver/doc/installation/robot_setup.rst)  
After finishing setting up the robot, you can run the launch file, and press the play button on the pendant.


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

__NOTE__: After launching the moveit_ur5_interface.launch, be sure to close the joint_state_publisher_gui before executing the motion of the robot. (The joint_state_publisher_gui is there just to avoid move_group complaining about missing joint state for the gripper.)

</br>

## PNP msgs

This package provides a custom action message type to request a pick-and-place action for the robot. 

There are three action messages in this package:

1. __`Robot.action`__:  
a. __Goal__: A geometry_msgs/Pose indicating the target pose of the robot
b. __Result__: A string that specifies a success or failure of the robot action. 
c. __Feedback__: A floating point value between 0 and 1 that represents the progress of the robot’s motion (0 at the beginning and 1 when completed).

2. __`Gripper.action`__:
a. __Goal__: A string that indicates if the gripper must open or close.
b. __Result__: A string that specifies a success or failure of the gripper action. 
c. __Feedback__: A floating point value between 0 and 1 that represents the progress of the gripper’s motion (0 at the beginning and 1 when completed).

3. __`PickAndPlace.action`__:  
a. __Goal__: There are two entries. One corresponds to a target pose where the robot must pick the object (a geometry_msgs/Pose that can be passed to Robot.action). The other corresponds to a target pose where the robot must place the object (also a geometry_msgs/Pose).  
b. __Result__: A string that specifies a success or failure of the pnp action.  
c. __Feedback__: A string that indicates if
    1. the robot is moving to pick the object 
    2. closing the gripper
    3. moving to place the object  
    4. opening the gripper.

</br>


## PNP actionlib
This package provides a action for initiating a pick-and-place task for the UR5 manipulator. 

### Launch Files

The package provides the following launch files:

1. __`ros2 launch pnp_actionlib ur5_pnp_actionlib_sim.launch`__  
This launch file is used when you don't have the actual UR5 and want to see it in simulation. It launches the `moveit_ur5_interface_sim.launch` and starts the pnp server that waits for action calls to initiate a pnp action service.
 
2. __`ros2 launch pnp_actionlib ur5_pnp_actionlib.launch`__  
This launch file is used when you want to actually control a real UR5. 
It launches the `moveit_ur5_interface.launch` and starts the pnp server that  waits for action calls to initiate a pnp action service.  
To connect to a real UR5, you can follow this instruction: [__Setting up a UR robot for ur_robot_driver__](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/galactic/ur_robot_driver/doc/installation/robot_setup.rst)  
After finishing setting up the robot, you can run the launch file, and press the play button on the pendant.


### Executables

The package provides the following executables:

1. __pnp_server.cpp__  
    ```
    ros2 run pnp_actinlib server
    ```
    This node initiates a action server waits for action calls to initiate a pnp action service.


2. __pnp_client.cpp__  
    ```
    ros2 run pnp_actinlib client
    ```  
    This is an example client node that sends a action request to the pnp_server.cpp. It constructs and sends a request to the server by calling the following function:
    ```
    pnp_call(const geometry_msgs::msg::Pose &pick_destination, const geometry_msgs::msg::Pose &place_destination)
    ```

### Usage

1. To send a pnp request from the command line, use the following commands:

    ```
    ros2 launch pnp_actionlib ur5_pnp_actionlib.launch

    ros2 action send_goal /pnp pnp_msgs/action/PickAndPlace "{pick_goal:{position: {x: 0.3, y: 0.4, z: 0.3}, orientation: {x: -1.0, y: 0.0, z: 0.0, w: 0.0}}, place_goal:{position: {x: -0.2, y: 0.4, z: 0.3}, orientation: {x: -1.0, y: 0.0, z: 0.0, w: 0.0}}}"
    ```

2. To send a plan request from the client node, modify the pick_goal and place_goal within the pnp_client.cpp file and use the following commands:

    ```
    ros2 launch pnp_actionlib ur5_pnp_actionlib.launch

    ros2 run pnp_actinlib client
    ```

3. To send a planning request within your own function, just include the following headers in your code:

    ```
    #include <pnp_actionlib/pnp_action.hpp>
    #include <pnp_actionlib/robot_action.hpp>
    #include <pnp_actionlib/gripper_action.hpp>
    ```
    Then, create a client node:
    ```
    auto pnpc = std::make_shared<rsp::pnp_client>("pnp");
    rclcpp::executors::MultiThreadedExecutor pnp_executor;
    pnp_executor.add_node(pnpc);
    ```
    Finally, call the pnp_call function and spin the node to start the pnp action:
    ```
    auto pnp_result = pnpc->pnp_call(pick_goal, place_goal);
    pnp_executor.spin_until_future_complete(pnp_result);
    ```

</br>

## Robotiq Description

This package basically is just the ROS2 version of the following package: [robotiq](https://github.com/filesmuggler/robotiq)  
It contains the urdf and meshes for the robotiq gripper. 

</br>

## UR5 Gripper Description

This package combines the urdf of UR5 with the robotiq gripper.
It attaches the gripper at the tool0 frame of the UR5. And a frame __ee_link__ was created at the center of the gripper so that we can use moveit2 to do cartesian planning with respect to this frame.

<p align="center">
<img src="https://github.com/JuoTungChen/ROS2_pick_and_place_UR5/blob/master/ur5_gripper_description/ur5_gripper.png" width="400">


</br>

## UR5 Gripper Moveit Config

This package contains the srdf for the UR5 with a robotiq gripper as a end effector. 

### Launch Files

1. __`ros2 launch ur5_gripper_moveit_config ur5_gripper_moveit.launch.py`__  
This launch file launches the moveit_config for the UR5 robot with a robotiq gripper as its end effector. Make sure you launch it after you launch the following launch file to spawn the controller for the robot. After that, you can use the interactive marker to drag around the end effector to desired pose and plan and execute the motion for the robot. 
    ```
    ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
    ```

