#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from tf2_ros import TransformBroadcaster
import math

class CustomFlightEngine(Node):
    def __init__(self):
        super().__init__('custom_flight_engine')
        
        # Klavyeden gelen komutları dinle
        self.sub = self.create_subscription(Twist, '/master/cmd_vel', self.cmd_cb, 10)
        
        # Gazebo'ya "Işınlanma/Konum Ayarlama" servisi (Fizik motorunu ezip geçer)
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # SLAM için Odom ve TF yayınlayıcılar (Söktüğümüz eklentinin görevini devralıyoruz)
        self.odom_pub = self.create_publisher(Odometry, '/master/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo bağlantısı bekleniyor...')

        # Anlık Hızlar
        self.v_x = 0.0
        self.v_yaw = 0.0

        # Master'ın Haritadaki Konumu
        self.m_x = 0.0
        self.m_y = 0.0
        self.m_yaw = 0.0
        self.z_height = 1.0 # 1 metre havada uçacaklar

        # Saniyede 30 kez çalışan Fizik Döngüsü
        self.timer = self.create_timer(1.0/30.0, self.update_physics)
        self.get_logger().info('Hollywood Uçuş Motoru Aktif! V-Formasyonu ve SLAM devrede.')

    def cmd_cb(self, msg):
        self.v_x = msg.linear.x
        self.v_yaw = msg.angular.z

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def set_drone_state(self, name, x, y, z, roll, pitch, yaw):
        req = SetEntityState.Request()
        req.state.name = name
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        req.state.pose.orientation.x = float(qx)
        req.state.pose.orientation.y = float(qy)
        req.state.pose.orientation.z = float(qz)
        req.state.pose.orientation.w = float(qw)
        self.cli.call_async(req)

    def update_physics(self):
        dt = 1.0 / 30.0

        # 1. Kinematik Hesaplama (Yeni konumu bul)
        self.m_yaw += self.v_yaw * dt
        self.m_x += self.v_x * math.cos(self.m_yaw) * dt
        self.m_y += self.v_x * math.sin(self.m_yaw) * dt

        # 2. Görsel Şov: İleri giderken burnunu eğ, dönerken yat
        pitch = 0.0
        roll = 0.0
        if self.v_x > 0.1:
            pitch = 0.15 # İleri eğil (~8 derece)
        elif self.v_x < -0.1:
            pitch = -0.15 # Geri yaslan
            
        if self.v_yaw > 0.1:
            roll = -0.15 # Sola yat
        elif self.v_yaw < -0.1:
            roll = 0.15 # Sağa yat

        # 3. Gazebo'daki Modelleri Taşı
        self.set_drone_state('master_uav', self.m_x, self.m_y, self.z_height, roll, pitch, self.m_yaw)
        
        # Slave 1 (Sol Arka Çapraz - V Formasyonu)
        s1_x = self.m_x - 1.0 * math.cos(self.m_yaw) - 1.0 * math.sin(self.m_yaw)
        s1_y = self.m_y - 1.0 * math.sin(self.m_yaw) + 1.0 * math.cos(self.m_yaw)
        self.set_drone_state('slave_uav_1', s1_x, s1_y, self.z_height, roll, pitch, self.m_yaw)

        # Slave 2 (Sağ Arka Çapraz - V Formasyonu)
        s2_x = self.m_x - 1.0 * math.cos(self.m_yaw) + 1.0 * math.sin(self.m_yaw)
        s2_y = self.m_y - 1.0 * math.sin(self.m_yaw) - 1.0 * math.cos(self.m_yaw)
        self.set_drone_state('slave_uav_2', s2_x, s2_y, self.z_height, roll, pitch, self.m_yaw)

        # 4. SLAM İçin Kusursuz Odometri ve TF Yayını (SLAM'in Kalbi)
        now = self.get_clock().now().to_msg()
        # Not: Haritanın (TF) yamulmaması için SLAM'e giden rotasyonda pitch ve roll sıfır kalır!
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.m_yaw) 

        # TF Yayını
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.m_x)
        t.transform.translation.y = float(self.m_y)
        t.transform.translation.z = float(self.z_height)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

        # Odometri Yayını
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.m_x)
        odom.pose.pose.position.y = float(self.m_y)
        odom.pose.pose.position.z = float(self.z_height)
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = float(self.v_x)
        odom.twist.twist.angular.z = float(self.v_yaw)
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = CustomFlightEngine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

