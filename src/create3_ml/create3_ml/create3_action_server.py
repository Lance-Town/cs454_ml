import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from create3_action_interfaces.action import ChangeState

from .movement import Slash

class ChangeStateActionServer(Node):
    def __init__(self):
        super().__init__('create3_change_state_action_server')

        self.robot = Slash() # Create the robot controller node

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

        state = goal_handle.request.state
        self.get_logger().info(f"GOAL STATE: {state}")
        success = False

        if (state == 1):
            success = self.robot.state_one(feedback_msg=feedback_msg, goal_handle=goal_handle) 
        elif (state == 2):
            success = self.robot.state_two(feedback_msg=feedback_msg, goal_handle=goal_handle)
        else:
            self.get_logger().info(f'State {state} not recognized')

        goal_handle.succeed()
        result = ChangeState.Result()
        
        result.success = success

        return result

def main(args=None):
    rclpy.init(args=args)

    change_state_action_server = ChangeStateActionServer()

    # 1 thread for subscription, another for action clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(change_state_action_server)
    exec.add_node(change_state_action_server.robot)

    try:
        exec.spin()
    except KeyboardInterrupt:
        change_state_action_server.get_logger().info('KeyboardInterrup, shutting down.')
    finally:
        change_state_action_server.destroy_node()
        change_state_action_server.robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()