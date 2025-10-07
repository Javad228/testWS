import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class BraitenbergNode(Node):
    
    LEFT_SECTOR_START_DEG = 30
    LEFT_SECTOR_END_DEG = 90
    FRONT_SECTOR_START_DEG = -30
    FRONT_SECTOR_END_DEG = 30
    RIGHT_SECTOR_START_DEG = -90
    RIGHT_SECTOR_END_DEG = -30

    def __init__(self):
        super().__init__('braitenberg_tb3')
        self.declare_parameter('gain', 1.2)
        self.declare_parameter('base_speed', 0.12)
        self.declare_parameter('turn_scale', 1.0)
        
        self.left_sensor_pub = self.create_publisher(Float32, '/braitenberg/left_min', 10)
        self.front_sensor_pub = self.create_publisher(Float32, '/braitenberg/front_min', 10)
        self.right_sensor_pub = self.create_publisher(Float32, '/braitenberg/right_min', 10)

        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def _get_sector_minimum(self, ranges, angle_min, angle_increment, start_angle_deg, end_angle_deg):
        num_ranges = len(ranges)
        start_rad = math.radians(start_angle_deg)
        end_rad = math.radians(end_angle_deg)
        
        start_index = max(0, int((start_rad - angle_min) / angle_increment))
        end_index = min(num_ranges - 1, int((end_rad - angle_min) / angle_increment))

        if start_index > end_index:
            start_index, end_index = end_index, start_index
            
        valid_ranges = [r for r in ranges[start_index:end_index+1] if not math.isinf(r) and not math.isnan(r)]
        return min(valid_ranges) if valid_ranges else 3.5

    def _calculate_repulsion(self, distance, gain):
        return gain / max(0.05, distance)

    def scan_callback(self, scan_data: LaserScan):
        if not scan_data.ranges:
            return

        gain = self.get_parameter('gain').get_parameter_value().double_value
        base_speed = self.get_parameter('base_speed').get_parameter_value().double_value
        turn_scale = self.get_parameter('turn_scale').get_parameter_value().double_value

        min_dist_left = self._get_sector_minimum(scan_data.ranges, scan_data.angle_min, scan_data.angle_increment,
                                                 self.LEFT_SECTOR_START_DEG, self.LEFT_SECTOR_END_DEG)
        min_dist_front = self._get_sector_minimum(scan_data.ranges, scan_data.angle_min, scan_data.angle_increment,
                                                  self.FRONT_SECTOR_START_DEG, self.FRONT_SECTOR_END_DEG)
        min_dist_right = self._get_sector_minimum(scan_data.ranges, scan_data.angle_min, scan_data.angle_increment,
                                                  self.RIGHT_SECTOR_START_DEG, self.RIGHT_SECTOR_END_DEG)

        left_repulsion = self._calculate_repulsion(min_dist_left, gain)
        front_repulsion = self._calculate_repulsion(min_dist_front, gain)
        right_repulsion = self._calculate_repulsion(min_dist_right, gain)

        linear_velocity = max(0.0, base_speed - 0.3 * front_repulsion)
        angular_velocity = turn_scale * (right_repulsion - left_repulsion)
        
        twist_command = Twist()
        twist_command.linear.x = linear_velocity
        twist_command.angular.z = angular_velocity

        self.left_sensor_pub.publish(Float32(data=float(min_dist_left)))
        self.front_sensor_pub.publish(Float32(data=float(min_dist_front)))
        self.right_sensor_pub.publish(Float32(data=float(min_dist_right)))

        self.velocity_pub.publish(twist_command)

def main():
    rclpy.init()
    node = BraitenbergNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
