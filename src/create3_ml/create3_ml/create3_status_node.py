import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import KidnapStatus
from scipy.spatial.transform import Rotation

class RobotStatusNode(Node):
    def __init__(self, namespace='create3_03EE'):
        super().__init__('robot_status_node')
        self.namespace = namespace

        qos = QoSProfile(depth=11)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.status_data = {
            'battery': None,
            'position': None,
            'kidnap': None
        }

        self.create_subscription(BatteryState, f'/{self.namespace}/battery_state', self.battery_callback, qos)
        self.create_subscription(Odometry, f'/{self.namespace}/odom', self.odom_callback, qos)
        self.create_subscription(KidnapStatus, f'/{self.namespace}/kidnap_status', self.kidnap_callback, qos)

    def battery_callback(self, msg):
        self.status_data['battery'] = msg.percentage
        self.get_logger().info(f'Battery: {(self.status_data["battery"]*100):.2f}%')

    def odom_callback(self, msg):
        self.status_data['position'] = {
                                        'position':
                                            {
                                                'x': msg.pose.pose.position.x,
                                                'y': msg.pose.pose.position.y,
                                                'z': msg.pose.pose.position.z
                                            },
                                        'quaternion':
                                            {
                                                'x': msg.pose.pose.orientation.x,
                                                'y': msg.pose.pose.orientation.y,
                                                'z': msg.pose.pose.orientation.z,
                                                'w': msg.pose.pose.orientation.w
                                            }
                                        }
        self.get_logger().info(f'{self.status_data["position"]}')

    def kidnap_callback(self, msg):
        self.status_data['kidnap'] = msg.is_kidnapped
        self.get_logger().info(f'Kidnap Status: {self.status_data["kidnap"]}')

    def get_status(self):
        return self.status_data

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()