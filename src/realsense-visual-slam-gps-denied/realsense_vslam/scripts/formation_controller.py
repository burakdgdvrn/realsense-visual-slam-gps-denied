#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import math

class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        
        self.sub_cmd = self.create_subscription(Twist, '/master/cmd_vel', self.cmd_cb, 10)
        
        # Hedef pozisyonları yayınlayacağımız kanallar
        self.pub_master = self.create_publisher(Pose, '/swarm/master_target', 10)
        self.pub_slave1 = self.create_publisher(Pose, '/swarm/slave1_target', 10)
        self.pub_slave2 = self.create_publisher(Pose, '/swarm/slave2_target', 10)

        # Durum Makinesi (State Machine): GROUND -> TAKEOFF -> HOVER -> FLYING
        self.state = 'GROUND'
        self.get_logger().info('Sürü Yerde Bekliyor. Kalkış için ileri komutu (i) verin.')

        self.v_x, self.v_yaw = 0.0, 0.0
        self.m_x, self.m_y, self.m_yaw, self.m_z = 0.0, 0.0, 0.0, 0.0

        self.timer = self.create_timer(1.0/30.0, self.update_formation)

    def cmd_cb(self, msg):
        self.v_x = msg.linear.x
        self.v_yaw = msg.angular.z

    def update_formation(self):
        dt = 1.0 / 30.0

        # --- DURUM MAKİNESİ (STATE MACHINE) ---
        if self.state == 'GROUND':
            if self.v_x > 0.0: # İlk komut geldiğinde kalkışa geç
                self.state = 'TAKEOFF'
                self.get_logger().info('Kalkış (Takeoff) Başladı!')
        
        elif self.state == 'TAKEOFF':
            self.m_z += 0.5 * dt # Saniyede 0.5m hızla yüksel
            if self.m_z >= 1.0:
                self.m_z = 1.0
                self.state = 'HOVER'
                self.get_logger().info('Havada Asılı (Hover). Formasyon Uçuşuna Hazır.')
                
        elif self.state == 'HOVER' or self.state == 'FLYING':
            if abs(self.v_x) > 0.0 or abs(self.v_yaw) > 0.0:
                self.state = 'FLYING'
            else:
                self.state = 'HOVER'
            
            # Kinematik Hareket
            self.m_yaw += self.v_yaw * dt
            self.m_x += self.v_x * math.cos(self.m_yaw) * dt
            self.m_y += self.v_x * math.sin(self.m_yaw) * dt

        # --- FORMASYON MATEMATİĞİ ---
        p_master = Pose()
        p_master.position.x, p_master.position.y, p_master.position.z = self.m_x, self.m_y, self.m_z
        # Oryantasyonu geçici olarak w ve z'ye saklıyoruz (Hız vektörlerini physics node'a iletmek için)
        p_master.orientation.w = self.m_yaw
        p_master.orientation.x = self.v_x # Hız bilgisini animasyon için yolluyoruz
        p_master.orientation.y = self.v_yaw

        p_s1 = Pose()
        p_s1.position.x = self.m_x - 1.0 * math.cos(self.m_yaw) - 1.0 * math.sin(self.m_yaw)
        p_s1.position.y = self.m_y - 1.0 * math.sin(self.m_yaw) + 1.0 * math.cos(self.m_yaw)
        p_s1.position.z = self.m_z
        p_s1.orientation = p_master.orientation

        p_s2 = Pose()
        p_s2.position.x = self.m_x - 1.0 * math.cos(self.m_yaw) + 1.0 * math.sin(self.m_yaw)
        p_s2.position.y = self.m_y - 1.0 * math.sin(self.m_yaw) - 1.0 * math.cos(self.m_yaw)
        p_s2.position.z = self.m_z
        p_s2.orientation = p_master.orientation

        # Yayınla
        self.pub_master.publish(p_master)
        self.pub_slave1.publish(p_s1)
        self.pub_slave2.publish(p_s2)

def main():
    rclpy.init()
    node = FormationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
