from std_srvs.srv import Trigger
import rclpy
from rclpy.node import Node

class TriggerClient(Node):
    def __init__(self):
        super().__init__('stdbot_trigger_client')
        self.cli = self.create_client(Trigger, '/Dave/change_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trigger service...')
        # self.req = Trigger.Request()

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# def main():
#     rclpy.init()
#     tc = TriggerClient()
#     tc.send_request()
#     # rclpy.spin()
#     tc.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()