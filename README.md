# setup

### Before you begin
You need to install and test the robot_description


## Run the following command to install:

```bash
$ sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gripper-controllers
$ cd robot_ws/src
$ git clone https://github.com/ZZSZSZSZZ/robot_ros2.git
$ cd ..
$ colcon build
$ source install/setup.bash
```

## Run the following command to control:

### Start the robot using fake hardware
```bash
$ ros2 launch robot_ros2_control robot.launch.py use_fake_hardware:=true
```
#### Parameters
* use_fake_hardware - Use fake hardware instead of real hardware (default: false)

### You can open a new terminal and test it with the following command
```bash
$ cd robot_ws
$ python3 src/robot_ros2/send_goal.py
```
