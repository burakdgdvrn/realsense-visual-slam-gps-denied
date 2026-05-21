#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
import random

class GpsBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_broadcaster')
        self.sub = self.create_subscription(Pose, '/swarm/master_target', self.odom_cb, 10)
        # ARTİK SADECE GPS YAYINLIYOR, TF YAYINLAMAYACAK
        self.odom_pub = self.create_publisher(Odometry, '/master/gps_odom', 10)
        
        # GPS Hata Modeli (Hocanın İstediği Gürültü ve Kayma)
        self.bias_x = 0.0
        self.bias_y = 0.0

    def euler_to_quaternion(self, yaw):
        return 0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)

    def odom_cb(self, msg):
        now = self.get_clock().now().to_msg()
        v_x, v_yaw = msg.orientation.x, msg.orientation.y
        qx, qy, qz, qw = self.euler_to_quaternion(msg.orientation.w)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 1. Random Walk Bias (Zamanla artan kayma)
        # Saniyede yaklaşık ~10 mesaj gelse, yavaş yavaş kayarak 3-4 metrelere ulaşır.
        self.bias_x += random.gauss(0.0, 0.02)
        self.bias_y += random.gauss(0.0, 0.02)
        
        # 2. Gaussian Noise (Anlık titreme)
        # Hocanın belirttiği gibi ~1.5 metre standart sapma
        noise_x = random.gauss(0.0, 1.5)
        noise_y = random.gauss(0.0, 1.5)

        # Kusursuz konuma gürültü ve kaymayı ekleyerek yayınla
        odom.pose.pose.position.x = msg.position.x + noise_x + self.bias_x
        odom.pose.pose.position.y = msg.position.y + noise_y + self.bias_y
        odom.pose.pose.position.z = msg.position.z

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_yaw
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = GpsBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()