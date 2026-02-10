import rclpy
from rclpy.action import ActionClient
from rclpy.time import Duration
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
 
class ControlActionClient(Node):
 
    def __init__(self):
        super().__init__('joint_trajectory_controller')
        self.client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_joint_trajectory_controller/follow_joint_trajectory'
        )

    def hello(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["arm_right_joint1", "arm_right_joint2", "arm_right_joint3", "arm_right_joint4", "arm_right_joint5", "arm_right_joint6", "arm_right_joint7"]
        point1 = JointTrajectoryPoint()
        point1.positions = [1.0, 0.8281, 1.9495, 1.25957, -0.896785, 0.0, 0.0]
        point1.time_from_start = Duration(seconds=2.0).to_msg()
        goal.trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [1.0, 0.8281, 1.247701, 1.775, -0.896785, -0.251328, 0.0]
        point2.time_from_start = Duration(seconds=4.0).to_msg()
        goal.trajectory.points.append(point2)

        point3 = JointTrajectoryPoint()
        point3.positions = [1.0, 0.8281, 1.9495, 1.25957, -0.896785, 0.0, 0.0]
        point3.time_from_start = Duration(seconds=6.0).to_msg()
        goal.trajectory.points.append(point3)

        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point4.time_from_start = Duration(seconds=8.0).to_msg()
        goal.trajectory.points.append(point4)

        point7 = JointTrajectoryPoint()
        point7.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point7.time_from_start = Duration(seconds=1.0).to_msg()
        #goal.trajectory.points.append(point7)
        
        

        return goal

    def send_goal(self):
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(self.hello())
 
def main(args=None):
    rclpy.init(args=args)
    action_client = ControlActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)
 
if __name__ == '__main__':
    main()
