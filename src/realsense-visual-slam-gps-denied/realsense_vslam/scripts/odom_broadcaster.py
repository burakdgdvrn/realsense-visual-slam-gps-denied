#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class OdomBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_broadcaster')
        self.sub = self.create_subscription(Pose, '/swarm/master_target', self.odom_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/master/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def euler_to_quaternion(self, yaw):
        return 0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)

    def odom_cb(self, msg):
        now = self.get_clock().now().to_msg()
        yaw = msg.orientation.w
        v_x = msg.orientation.x
        v_yaw = msg.orientation.y
        qx, qy, qz, qw = self.euler_to_quaternion(yaw)

        # SLAM için TF (Eğim ve yatma olmadan, saf konum)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf_broadcaster.sendTransform(t)

        # Odom
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position = msg.position
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_yaw
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
