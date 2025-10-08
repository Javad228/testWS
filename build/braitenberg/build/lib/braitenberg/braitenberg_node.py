import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Robot(Node):
    
    LEFTA = 30
    LEFTB = 90
    FRNTA = -30
    FRNTB = 30
    RGHTA = -90
    RGHTB = -30

    def __init__(self):
        super().__init__('braitenberg')
        self.declare_parameter('gain', 1.2)
        self.declare_parameter('turn_scale', 1.0)
        
        self.pub_l = self.create_publisher(Float32, '/braitenberg/left_min', 10)
        self.pub_f = self.create_publisher(Float32, '/braitenberg/front_min', 10)
        self.pub_r = self.create_publisher(Float32, '/braitenberg/right_min', 10)

        self.pub_v = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_s = self.create_subscription(LaserScan, '/scan', self.onscn, 10)

    def get_s(self, rnges, ang_m, ang_i, s_deg, e_deg):
        n_rng = len(rnges)
        
        s_rad = math.radians(s_deg)
        e_rad = math.radians(e_deg)
        
        s_idx = int((s_rad - ang_m) / ang_i) % n_rng
        e_idx = int((e_rad - ang_m) / ang_i) % n_rng

        if s_idx > e_idx:
            vals1 = [v for v in rnges[s_idx:] if not math.isinf(v) and not math.isnan(v)]
            vals2 = [v for v in rnges[:e_idx+1] if not math.isinf(v) and not math.isnan(v)]
            vals_ = vals1 + vals2
        else:
            vals_ = [v for v in rnges[s_idx:e_idx+1] if not math.isinf(v) and not math.isnan(v)]
            
        return min(vals_) if vals_ else 3.5

    def cal_r(self, dist_, gain_):
        return gain_ / max(0.05, dist_)

    def onscn(self, sdata: LaserScan):
        if not sdata.ranges:
            return

        gain_ = self.get_parameter('gain').get_parameter_value().double_value
        t_scl = self.get_parameter('turn_scale').get_parameter_value().double_value

        l_dst = self.get_s(sdata.ranges, sdata.angle_min, sdata.angle_increment,
                                                 self.LEFTA, self.LEFTB)
        f_dst = self.get_s(sdata.ranges, sdata.angle_min, sdata.angle_increment,
                                                  self.FRNTA, self.FRNTB)
        r_dst = self.get_s(sdata.ranges, sdata.angle_min, sdata.angle_increment,
                                                  self.RGHTA, self.RGHTB)

        l_rep = self.cal_r(l_dst, gain_)
        f_rep = self.cal_r(f_dst, gain_)
        r_rep = self.cal_r(r_dst, gain_)

        l_vel = max(0.0, 0.12 - 0.3 * f_rep)
        a_vel = t_scl * (r_rep - l_rep)
        
        t_cmd = Twist()
        t_cmd.linear.x = l_vel
        t_cmd.angular.z = a_vel

        self.pub_l.publish(Float32(data=float(l_dst)))
        self.pub_f.publish(Float32(data=float(f_dst)))
        self.pub_r.publish(Float32(data=float(r_dst)))

        self.pub_v.publish(t_cmd)

def runit():
    rclpy.init()
    anode = Robot()
    try:
        rclpy.spin(anode)
    except KeyboardInterrupt:
        pass
    finally:
        anode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    runit()
