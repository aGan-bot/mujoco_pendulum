#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class PendulumTester(Node):
    def __init__(self):
        super().__init__('pendulum_tester')

        # Effort komutlarını göndereceğimiz publisher
        self.effort_pub = self.create_publisher(Float64, '/effort_controller/commands', 10)

        # Joint durumlarını dinleyeceğimiz subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Timer ile düzenli kuvvet gönder
        self.timer = self.create_timer(0.1, self.send_effort)
        self.effort_value = 1.0
        self.step = 0

    def joint_callback(self, msg):
        pos = msg.position[0] if msg.position else 0.0
        vel = msg.velocity[0] if msg.velocity else 0.0
        eff = msg.effort[0] if msg.effort else 0.0
        self.get_logger().info(f'Position: {pos:.3f}, Velocity: {vel:.3f}, Effort: {eff:.3f}')

    def send_effort(self):
        # Alternatif kuvvetle sallıyoruz
        self.effort_pub.publish(Float64(data=self.effort_value))
        self.step += 1
        if self.step % 20 == 0:
            self.effort_value *= -1  # 2 saniyede bir yön değiştir

def main(args=None):
    rclpy.init(args=args)
    node = PendulumTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()