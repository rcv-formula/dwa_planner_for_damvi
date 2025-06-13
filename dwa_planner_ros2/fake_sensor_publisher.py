#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # scan parameters
        self.angle_min = -math.pi / 2
        self.angle_max =  math.pi / 2
        self.angle_increment = math.pi / 180  # 1°
        self.range_min = 0.2
        self.range_max = 5.0

    def timer_callback(self):
        self.publish_scan()
        self.publish_odom()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        msg.ranges = []
        for i in range(num_readings):
            angle = self.angle_min + i * self.angle_increment
            # 정면 ±10° 구역에 고정 장애물 0.5m
            if abs(angle) < math.radians(10):
                msg.ranges.append(0.5)
            else:
                msg.ranges.append(3.0)

        self.scan_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # 로봇은 정지 상태
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.angular.z = 0.0
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
