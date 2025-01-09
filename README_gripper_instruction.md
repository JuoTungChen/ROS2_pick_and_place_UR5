# Instruction to install and run Robotiq gripper driver

## Installation
### Clone robotiq repo
Start the command from the root workspace directory.
```
cd src
git clone https://github.com/sequenceplanner/robotiq_2f.git
```

### Install ros2_rust
For detail instruction, please follow this [link](https://github.com/ros2-rust/ros2_rust/blob/main/docs/building.md).
Here are all the commands:
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
git clone https://github.com/ros2-rust/ros2_rust.git
sudo apt install -y git libclang-dev python3-pip python3-vcstool
cargo install cargo-ament-build
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git
```
Now we need to modify the .repos file because we are using ROS2 galactic. Make a copy of `ros2_rust_foxy.repos` and named it `ros2_rust_galactic.repos`. Modify all `foxy` into `galactic` in the file.
```
cd .. (move back to root workspace directory)
vcs import src < src/ros2_rust/ros2_rust_galactic.repos
```

### Build robotiq_2f_driver
```
colcon build --packages-up-to robotiq_2f_driver
colcon build --packages-up-to robotiq_2f_driver_ui
```

## Running Robotiq gripper driver
### Find USB device name
I have tried `lsusb` and `lsblk`, but they don't work well. Run this command instead:
```
dmesg
```
Hopefully, you will see something like `ttyUSB0`. And now we need to give permission to access it:
```
sudo chmod 777 /dev/ttyUSB0
```

### Launching the driver
You will see the gripper close and open.
```
. install/setup.bash
ros2 run robotiq_2f_driver robotiq_2f_driver
```

### Controlling the gripper from command
```
. install/setup.bash
ros2 topic pub -1 /robotiq_2f_command robotiq_2f_msgs/msg/CommandState "{command: 'close'}"
ros2 topic pub -1 /robotiq_2f_command robotiq_2f_msgs/msg/CommandState "{command: 'open'}"
```

### Controlling the gripper with GUI
There will be a window with open and close buttons.
```
. install/setup.bash
ros2 run robotiq_2f_driver_ui robotiq_2f_driver_ui
```
