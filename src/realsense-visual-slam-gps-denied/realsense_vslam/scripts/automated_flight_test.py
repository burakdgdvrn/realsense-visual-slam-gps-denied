#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutomatedFlightTest(Node):
    def __init__(self):
        super().__init__('automated_flight_test')
        self.publisher_ = self.create_publisher(Twist, '/master/cmd_vel', 10)
        
        # 2 saniye bekle simülasyon kendine gelsin
        time.sleep(2)
        
        self.get_logger().info('OTOMATİK UÇUŞ TESTİ BAŞLIYOR...')
        self.run_test_sequence()

    def publish_cmd(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher_.publish(msg)
            time.sleep(0.1) # 10Hz ile yayınla

    def run_test_sequence(self):
        # 1. KALKIŞ (Takeoff tetikleyici: azıcık ileri komutu ver)
        self.get_logger().info('1. AŞAMA: Kalkış Tetikleniyor...')
        self.publish_cmd(0.1, 0.0, 1.0) # Sadece kalkışı tetiklemek için ufak komut
        
        self.get_logger().info('1. AŞAMA: Kalkış ve Hover bekleniyor (5 sn)...')
        self.publish_cmd(0.0, 0.0, 5.0) # Havaya kalkıp asılı kalması için süre tanı
        
        # 2. DÜZ UÇUŞ
        self.get_logger().info('2. AŞAMA: Düz Uçuş (10 sn)...')
        self.publish_cmd(0.5, 0.0, 10.0)
        
        # 3. DURUŞ VE BEKLEME
        self.get_logger().info('3. AŞAMA: Dur ve Havada Asılı Kal (Hover - 3 sn)...')
        self.publish_cmd(0.0, 0.0, 3.0)
        
        # 4. KENDİ EKSENİNDE DÖNÜŞ
        self.get_logger().info('4. AŞAMA: Sola Dönüş (90 derece civarı - 4 sn)...')
        self.publish_cmd(0.0, 0.4, 4.0)
        
        # 5. DÖNÜŞ SONRASI DURUŞ
        self.get_logger().info('5. AŞAMA: Dur ve Havada Asılı Kal (Hover - 2 sn)...')
        self.publish_cmd(0.0, 0.0, 2.0)
        
        # 6. YENİ YÖNE DOĞRU UÇUŞ
        self.get_logger().info('6. AŞAMA: Yeni Yöne Düz Uçuş (8 sn)...')
        self.publish_cmd(0.5, 0.0, 8.0)
        
        # 7. GÖREV BİTİŞİ
        self.get_logger().info('7. AŞAMA: Uçuş Tamamlandı. Sürü Hover Konumunda Bekliyor.')
        self.publish_cmd(0.0, 0.0, 2.0)
        
        self.get_logger().info('TEST BAŞARIYLA BİTTİ.')

def main(args=None):
    rclpy.init(args=args)
    test_node = AutomatedFlightTest()
    # Script bitince node'u öldürmeye gerek yok, sequence bitince kendi çıkacak.
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
