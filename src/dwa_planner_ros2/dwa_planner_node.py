import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from dwa_planner_ros2.parameters import declare_parameters
from dwa_planner_ros2.dwa_planner import DWAPlanner

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        declare_parameters(self)
        params = {p.name: p.value for p in self.list_parameters('', 1).parameters}
        self.planner = DWAPlanner(params)

        self.obs_list = []
        self.goal = None
        self.current_twist = Twist()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finish_pub = self.create_publisher(Bool, 'finish_flag', 1)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_cb, 1)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 1)

        timer_period = 1.0 / params['hz']
        self.create_timer(timer_period, self.timer_cb)

    def goal_cb(self, msg):
        self.goal = msg.pose

    def odom_cb(self, msg):
        self.current_twist = msg.twist.twist

    def scan_cb(self, msg):
        self.obs_list = []
        angle = msg.angle_min
        step = int(self.planner.angle_resolution / msg.angle_increment)
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and i % step == 0:
                pt = Point(x=r*math.cos(angle), y=r*math.sin(angle))
                self.obs_list.append(pt)
            angle += msg.angle_increment

    def timer_cb(self):
        if not self.goal:
            return
        best = self.planner.dwa_planning(self.current_twist, self.goal, self.obs_list)
        if best:
            cmd = Twist()
            cmd.linear.x = best[0]['velocity']
            cmd.angular.z = best[0]['yawrate']
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
