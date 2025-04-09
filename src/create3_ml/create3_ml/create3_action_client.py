import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from create3_action_interfaces.action import ChangeState
from action_msgs.msg._goal_status import GoalStatus

class ChangeStateActionClient(Node):
    def __init__(self):
        super().__init__('create3_change_state_action_client')
        self._action_client = ActionClient(self, ChangeState, 'change_state')

    def send_goal(self, state):
        goal_msg = ChangeState.Goal()
        goal_msg.state = state

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal=goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # while not self._send_goal_future.done():
        #     pass
        
        # self._goal_uuid = self._send_goal_future.result()

        # while self._goal_uuid and self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
        #     pass

        # while self._goal_uuid and self._goal_uuid.status is not GoalStatus.STATUS_SUCCEEDED:
        #     if self._goal_uuid is None or self._goal_uuid.status is GoalStatus.STATUS_CANCELED:
        #         break
        #     pass
        
        # self._goal_uuid = None

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result {result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Recieved Feedback: {feedback.percent_finished}')

def main(args=None):
    rclpy.init(args=args)

    action_client = ChangeStateActionClient()

    action_client.send_goal(1)
    action_client.send_goal(2)

    # rclpy.spin(action_client)
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('KeyboardInterrup, shutting down.')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()