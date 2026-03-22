#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        # Master'ın hareket komutlarını dinle
        self.subscription = self.create_subscription(
            Twist, '/master/cmd_vel', self.listener_callback, 10)
        
        # Slave'lere aynı komutları yayınla
        self.pub_slave1 = self.create_publisher(Twist, '/slave_uav_1/cmd_vel', 10)
        self.pub_slave2 = self.create_publisher(Twist, '/slave_uav_2/cmd_vel', 10)
        
        self.get_logger().info('Sürü Zekası Aktif! Slave İHA-1 ve İHA-2, Master İHA\'ya kilitlendi.')

    def listener_callback(self, msg):
        # Master ne yaparsa (ileri, geri, dön), Slave'ler de aynısını yapsın
        self.pub_slave1.publish(msg)
        self.pub_slave2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
