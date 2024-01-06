import sys
import rclpy as rp
from rclpy.node import Node

from rclpy.action import ActionClient
from dobot_ros2_interface.action import Move

class MoveClient(Node):

    def __init__(self):
        super().__init__('move_goal_client')
        self.move_client = ActionClient(self, Move, 'move_goal_server')


    def send_goal(self, goal_pose):
        goal_msg = Move.Goal()
        goal_msg.goal_pose.position.x = float(goal_pose[0])
        goal_msg.goal_pose.position.y = float(goal_pose[1])
        goal_msg.goal_pose.position.z = float(goal_pose[2])
        goal_msg.goal_pose.orientation.w = 0.0
        self.move_client.wait_for_server()

        self.send_goal_future = self.move_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
         
    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result_str))
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_pose))



def main(args=None):
    rp.init(args=args)
    client = MoveClient()
    args = sys.argv[1:]
    client.send_goal(args)

    rp.spin_once(client)

    client.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()