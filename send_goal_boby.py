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
            self, FollowJointTrajectory, '/body_joint_trajectory_controller/follow_joint_trajectory'
        )

    def move(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["body_joint1"]

        point1 = JointTrajectoryPoint()
        point1.positions = [0.2]
        point1.time_from_start = Duration(seconds=0.0).to_msg()
        goal.trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.0]
        point2.time_from_start = Duration(seconds=1.0).to_msg()
        #goal.trajectory.points.append(point2)

        return goal

    def send_goal(self):
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(self.move())
 
def main(args=None):
    rclpy.init(args=args)
    action_client = ControlActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)
 
if __name__ == '__main__':
    main()
