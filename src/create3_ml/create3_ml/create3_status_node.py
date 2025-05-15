import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import KidnapStatus
from irobot_create_msgs.msg import DockStatus
from scipy.spatial.transform import Rotation
# from numpy import uint16

class RobotStatusNode(Node):
    def __init__(self, namespace='create3_03EE'):
        super().__init__(f'{namespace}_status_node')
        self.namespace = namespace

        qos = QoSProfile(depth=11)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.status_data = {
            'battery': None,
            'position': None,
            'kidnap': None,
            'dock': None
        }

        self.create_subscription(BatteryState, f'/{self.namespace}/battery_state', self.battery_callback, qos)
        self.create_subscription(Odometry, f'/{self.namespace}/odom', self.odom_callback, qos)
        self.create_subscription(KidnapStatus, f'/{self.namespace}/kidnap_status', self.kidnap_callback, qos)
        self.create_subscription(DockStatus, f'{namespace}/dock_status', self.docked_callback, qos)

    def battery_callback(self, msg):
        self.status_data['battery'] = (msg.percentage*100)
        self.get_logger().info(f'Battery: {(self.status_data["battery"])}%')

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

    def kidnap_callback(self, msg):
        self.status_data['kidnap'] = msg.is_kidnapped
    
    def docked_callback(self, msg):
        self.status_data['dock'] = msg.is_docked

    def get_status(self):
        return self.status_data

    def get_battery(self) -> int:
        if (self.status_data["battery"] is None):
            return -1
        return self.status_data["battery"]
    
    def get_position(self):
        if (self.status_data["position"] is None):
            return [0,0,0]
        else:
            return [int(self.status_data['position']['position']['x']), int(self.status_data['position']['position']['y'])]

    def get_dock_status(self) -> bool:
        if (self.status_data["dock"] is None):
            return True
        return self.status_data["dock"]
        
    def get_degree_angle(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyBoard Interrupt. Shutting Down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()