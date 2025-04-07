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

        # TODO depending on the state requested, do some work. Use an if statement most likely
        # TODO I probably don't need the other create3_ml package. That one was going to contain the movement code, but I dont think I am going
        # to need that. Really, I will include the irobot_create3 package as a dependecy of this one. Then I can create the movement.py that will 
        # have the movements. This (i dont think) will need to be hosted on a node. It should just be able to send the movements without needing
        # to be on its own node. Then we can do something like
        # 
        # if (state == 1):
        #   self.movement.perform_state_one(feedback_msg, goal_handle) 


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