import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from create3_action_interfaces.action import ChangeState

class ChangeStateActionClient(Node):
    def __init__(self):
        super().__init__('create3_change_state_action_client')
        self._action_client = ActionClient(self, ChangeState, 'change_state')

    def send_goal(self, state):
        goal_msg = ChangeState.Goal()
        goal_msg.state = state

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    action_client = ChangeStateActionClient()

    future = action_client.send_goal(5)

    rclpy.spin_until_future_complete(action_client, future=future)

if __name__ == '__main__':
    main()