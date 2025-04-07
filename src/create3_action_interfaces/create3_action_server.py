import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from create3_action_interfaces.action import ChangeState

class ChangeStateActionServer(Node):
    def __init__(self):
        super().__init__('create3_change_state_action_server')
        self._action_server = ActionServer(
            self,
            ChangeState,
            'change_state',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = ChangeState.Feedback()
        feedback_msg.percent_finished = 0.0

        for i in range(10):
            feedback_msg.percent_finished = (i+1)*10.0
            self.get_logger().info(f'Feedback: {feedback_msg.percent_finished}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = ChangeState.Result()
        
        if feedback_msg.percent_finished == 100:
            result.success = True
        else:
            result.success = False

        return result

def main(args=None):
    rclpy.init(args=args)

    change_state_action_server = ChangeStateActionServer()

    rclpy.spin(change_state_action_server)

if __name__ == '__main__':
    main()