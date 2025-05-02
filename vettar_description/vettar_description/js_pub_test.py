import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisherTest(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(3.0, self.publish_joint_state)  # 3초 주기로 발행
        self.joint_angle = 0
        self.get_logger().info("start")

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['F_L_0', 'F_L_1', 'F_L_2', 'F_R_3', 'F_R_4', 'F_R_5',
                            'B_L_6', 'B_L_7', 'B_L_8', 'B_R_9', 'B_R_10', 'B_R_11']

        if self.joint_angle == 0:
            joint_state.position = [math.radians(95.6), math.radians(80.05), math.radians(28.52),
                                    math.radians(84.4), math.radians(99.95), math.radians(151.48),
                                    math.radians(84.4), math.radians(121.98), math.radians(70.45),
                                    math.radians(95.6), math.radians(58.02), math.radians(109.55)]
            self.joint_angle = 1
        else:
            joint_state.position = [math.radians(90.0), math.radians(82.45), math.radians(60.42),
                                    math.radians(90.0), math.radians(97.55), math.radians(119.58),
                                    math.radians(90.0), math.radians(90.08), math.radians(68.05),
                                    math.radians(90.0), math.radians(89.92), math.radians(111.95)]
            self.joint_angle = 0

        self.publisher_.publish(joint_state)
        self.get_logger().info(f"joint state: {joint_state.position}")

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisherTest()
    rclpy.spin(joint_state_publisher)

if __name__ == '__main__':
    main()

