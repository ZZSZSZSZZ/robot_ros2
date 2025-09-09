### Run the following command to install:

```bash

$ sudo apt-get install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gripper-controllers
$ cd robot_ws/src
$ git clone https://github.com/ZZSZSZSZZ/robot_ros2.git
$ cd ..
$ colcon build
$ source install/setup.bash
$ ros2 launch robot_ros2_control robot.launch.py
```

### Run the following command to test:

```bash
$ ros2 action send_goal /right_arm_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory '{trajectory: {joint_names: ["arm_right_joint1"], points: [{positions: [0.5], time_from_start: {sec: 1, nanosec: 0}}]}}'
```
or Open a new terminal

```bash
$ python3 src/robot_ros2/send_goal.py
```
