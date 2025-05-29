import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from dwa_planner_ros2.parameters import Parameters
from dwa_planner_ros2.dwa_planner import DWAPlanner

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        # 파라미터 로드 & 출력
        self.params = Parameters(self)
        self.params.print(self)

        # Planner
        self.planner = DWAPlanner(self.params)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 상태 변수
        self.goal       = None
        self.obs_list   = []
        self.path_poses = []
        self.current_twist = None
        self.finish_flag = Bool()

        # 퍼블리셔
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finish_pub = self.create_publisher(Bool, 'finish_flag', 1)
        self.cand_pub   = self.create_publisher(MarkerArray, 'candidate_trajectories', 1)
        self.sel_pub    = self.create_publisher(Marker, 'selected_trajectory', 1)
        self.fp_pub     = self.create_publisher(MarkerArray, 'predict_footprints', 1)

        # 구독자
        self.create_subscription(PoseStamped,      '/move_base_simple/goal', self.goal_cb, 1)
        self.create_subscription(Path,             '/path',                  self.path_cb, 1)
        self.create_subscription(LaserScan,        '/scan',                  self.scan_cb, 1)
        self.create_subscription(OccupancyGrid,    '/local_map',             self.map_cb, 1)
        self.create_subscription(Odometry,         '/odom',                  self.odom_cb, 1)
        self.create_subscription(Twist,            '/target_velocity',       self.tv_cb, 1)
        self.create_subscription(Bool,             '/dist_to_goal_th',       self.dist_cb, 1)

        # 메인 루프 타이머
        self.create_timer(1.0 / self.params.hz, self.timer_cb)

    # 콜백 정의
    def goal_cb(self, msg):
        self.goal = msg

    def path_cb(self, msg):
        self.path_poses = [p.pose.position for p in msg.poses]

    def scan_cb(self, msg):
        self.obs_list = []
        angle = msg.angle_min
        step  = int(self.params.angle_resolution / msg.angle_increment)
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and i % step == 0:
                self.obs_list.append(Point(x=r*math.cos(angle), y=r*math.sin(angle)))
            angle += msg.angle_increment

    def map_cb(self, grid):
        self.obs_list = []
        max_d = math.hypot(grid.info.width*grid.info.resolution,
                           grid.info.height*grid.info.resolution)
        a = grid.info.origin.position.x
        b = grid.info.origin.position.y
        for ang in np.arange(-math.pi, math.pi, self.params.angle_resolution):
            for dist in np.arange(0, max_d, grid.info.resolution):
                x = dist*math.cos(ang) + a
                y = dist*math.sin(ang) + b
                ix = int((x - a) / grid.info.resolution)
                iy = int((y - b) / grid.info.resolution)
                if 0 <= ix < grid.info.width and 0 <= iy < grid.info.height:
                    if grid.data[ix + iy*grid.info.width] == 100:
                        self.obs_list.append(Point(x=x, y=y))
                        break

    def odom_cb(self, msg):
        self.current_twist = msg.twist.twist

    def tv_cb(self, msg):
        self.params.target_velocity = min(msg.linear.x, self.params.max_velocity)

    def dist_cb(self, msg):
        self.params.dist_to_goal_th = msg.data

    # 주기 실행
    def timer_cb(self):
        if not self.goal or not self.current_twist:
            return

        # 목표를 로봇 프레임으로 변환
        try:
            t = self.tf_buffer.lookup_transform(self.params.robot_frame,
                                                self.goal.header.frame_id,
                                                rclpy.time.Time())
            transformed = do_transform_pose(self.goal, t)
            goal_pos = transformed.pose.position
        except Exception:
            goal_pos = self.goal.pose.position

        # DWA 계획
        best_traj = self.planner.dwa_planning(self.current_twist,
                                              goal_pos,
                                              self.obs_list,
                                              self.path_poses)

        # 명령 속도 퍼블리시
        cmd = Twist()
        cmd.linear.x  = best_traj[0].velocity
        cmd.angular.z = best_traj[0].yawrate
        self.cmd_pub.publish(cmd)
        self.finish_pub.publish(self.finish_flag)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
