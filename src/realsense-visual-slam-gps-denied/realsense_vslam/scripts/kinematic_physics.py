#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetEntityState
import math

class KinematicPhysics(Node):
    def __init__(self):
        super().__init__('kinematic_physics')
        
        self.sub_m = self.create_subscription(Pose, '/swarm/master_target', self.master_cb, 10)
        self.sub_s1 = self.create_subscription(Pose, '/swarm/slave1_target', self.s1_cb, 10)
        self.sub_s2 = self.create_subscription(Pose, '/swarm/slave2_target', self.s2_cb, 10)
        
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo Fizik Motoruna Bağlanılıyor...')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def move_drone(self, name, pose_msg):
        v_x = pose_msg.orientation.x
        v_yaw = pose_msg.orientation.y
        yaw = pose_msg.orientation.w
        
        # Aerodinamik Animasyon
        pitch, roll = 0.0, 0.0
        if v_x > 0.1: pitch = 0.15
        elif v_x < -0.1: pitch = -0.15
        if v_yaw > 0.1: roll = -0.15
        elif v_yaw < -0.1: roll = 0.15

        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = pose_msg.position.x
        req.state.pose.position.y = pose_msg.position.y
        req.state.pose.position.z = pose_msg.position.z
        
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        self.cli.call_async(req)

    def master_cb(self, msg): self.move_drone('master_uav', msg)
    def s1_cb(self, msg): self.move_drone('slave_uav_1', msg)
    def s2_cb(self, msg): self.move_drone('slave_uav_2', msg)

def main():
    rclpy.init()
    node = KinematicPhysics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
