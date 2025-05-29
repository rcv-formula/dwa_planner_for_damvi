import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FakeGoalPublisher(Node):
    def __init__(self):
        super().__init__('fake_goal_publisher')
        self.pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)  # 1 Hz

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 목표 위치를 로봇 뒤쪽(-1.0m)으로 설정
        msg.pose.position.x = -1.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
