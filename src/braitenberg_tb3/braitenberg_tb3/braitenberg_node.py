import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class BraitenbergNode(Node):
    def __init__(self):
        super().__init__('braitenberg_tb3')
        self.declare_parameter('gain', 1.2)         # obstacle sensitivity
        self.declare_parameter('base_speed', 0.12)  # forward speed m/s
        self.declare_parameter('turn_scale', 1.0)   # turn aggressiveness
        self.pub_left  = self.create_publisher(Float32, '/braitenberg/left_min', 10)
        self.pub_front = self.create_publisher(Float32, '/braitenberg/front_min', 10)
        self.pub_right = self.create_publisher(Float32, '/braitenberg/right_min', 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        def sector_min(start_deg, end_deg):
            start = math.radians(start_deg)
            end = math.radians(end_deg)
            i0 = max(0, int((start - msg.angle_min) / msg.angle_increment))
            i1 = min(n - 1, int((end - msg.angle_min) / msg.angle_increment))
            if i0 > i1:
                i0, i1 = i1, i0
            vals = [r for r in msg.ranges[i0:i1+1] if not math.isinf(r) and not math.isnan(r)]
            return min(vals) if vals else 3.5

        left_min  = sector_min(30, 90)
        front_min = sector_min(-30, 30)
        right_min = sector_min(-90, -30)

        gain = float(self.get_parameter('gain').get_parameter_value().double_value)
        base_speed = float(self.get_parameter('base_speed').get_parameter_value().double_value)
        turn_scale = float(self.get_parameter('turn_scale').get_parameter_value().double_value)

        def repulse(d):
            d = max(0.05, d)
            return gain / d

        rep_l = repulse(left_min)
        rep_f = repulse(front_min)
        rep_r = repulse(right_min)

        linear = max(0.0, base_speed - 0.3 * rep_f)
        angular = turn_scale * (rep_r - rep_l)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub_left.publish(Float32(data=float(left_min)))
        self.pub_front.publish(Float32(data=float(front_min)))
        self.pub_right.publish(Float32(data=float(right_min)))

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = BraitenbergNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
