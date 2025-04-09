import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from create3_action_interfaces.action import ChangeState

class ChangeStateActionClient(Node):
    def __init__(self):
        super().__init__('create3_change_state_action_client')
        self._action_client = ActionClient(self, ChangeState, 'change_state')

    def send_goal_and_wait(self, state):
        goal_msg = ChangeState.Goal()
        goal_msg.state = state

        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f'Goal {state} rejected')
            return

        self.get_logger().info(f'Goal {state} accepted')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        self.get_logger().info(f'Goal {state} result: {result.success}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received Feedback: {feedback.percent_finished}')

def main(args=None):
    rclpy.init(args=args)

    action_client = ChangeStateActionClient()

    try:
        # Goal 1
        action_client.send_goal_and_wait(1)

        # Goal 2 (only starts after goal 1 is done)
        action_client.send_goal_and_wait(2)

    except KeyboardInterrupt:
        action_client.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
