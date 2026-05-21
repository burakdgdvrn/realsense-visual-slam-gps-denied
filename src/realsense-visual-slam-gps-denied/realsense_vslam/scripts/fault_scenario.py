#!/usr/bin/env python3
"""
Otomatik Arıza Senaryosu — GPS Kesintili Tam Test

Senaryo Akışı:
  1. Kalkış + Hover (6 sn)
  2. Düz uçuş — GPS aktif (10 sn)
  3. 🔴 GPS KESİNTİSİ — Siber saldırı simülasyonu
  4. GPS'siz düz uçuş — VO ile devam (10 sn)
  5. GPS'siz dönüş — VO ile dönüş (4 sn)
  6. GPS'siz yeni yöne uçuş (6 sn)
  7. 🟢 GPS GERİ GELDİ — Yumuşak geçiş
  8. GPS ile düz uçuş — Recovery sonrası (8 sn)
  9. Hover ve görev bitişi (3 sn)

Bu script hem uçuş komutlarını hem GPS durumunu otomatik kontrol eder.
Manuel müdahale gerekmez.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
import time


class FaultScenario(Node):
    def __init__(self):
        super().__init__('fault_scenario')
        self.cmd_pub = self.create_publisher(Twist, '/master/cmd_vel', 10)
        
        # İlk mesajın kaçmaması için (Latching) QoS Profili
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.gps_pub = self.create_publisher(Bool, '/system/gps_status', qos_profile)

        # Başlangıçta GPS aktif
        self.set_gps(True)

        # Simülasyonun hazır olmasını bekle
        time.sleep(2)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  🚁  ARIZA SENARYO TESTİ BAŞLIYOR  🚁')
        self.get_logger().info('=' * 50)

        self.run_scenario()

    def publish_cmd(self, linear_x, angular_z, duration):
        """Belirtilen süre boyunca hız komutu yayınla"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_pub.publish(msg)
            time.sleep(0.1)

    def set_gps(self, active):
        """GPS durumunu değiştir"""
        msg = Bool()
        msg.data = active
        # Birkaç kez yayınla ki mesaj kaybolmasın
        for _ in range(5):
            self.gps_pub.publish(msg)
            time.sleep(0.05)

    def run_scenario(self):
        # ===== FAZA 1: KALKIŞ =====
        self.get_logger().info('📍 FAZA 1: Kalkış Tetikleniyor...')
        self.publish_cmd(0.1, 0.0, 1.0)
        self.get_logger().info('📍 FAZA 1: Hover Bekleniyor (5 sn)...')
        self.publish_cmd(0.0, 0.0, 5.0)

        # ===== FAZA 2: GPS İLE DÜZ UÇUŞ =====
        self.get_logger().info('📍 FAZA 2: GPS ile Düz Uçuş (10 sn)...')
        self.publish_cmd(0.5, 0.0, 10.0)

        # ===== FAZA 3: GPS KESİNTİSİ =====
        self.get_logger().info('')
        self.get_logger().info('🔴' * 25)
        self.get_logger().info('🔴  SİMÜLE EDİLEN KESİNTİ: GPS SİNYALİ KOPARILIYOR!  🔴')
        self.get_logger().info('🔴' * 25)
        self.get_logger().info('')
        self.set_gps(False)

        # Kısa hover — VO'nun stabilize olmasını bekle
        self.get_logger().info('📍 FAZA 3: VO Stabilizasyonu Bekleniyor (3 sn)...')
        self.publish_cmd(0.0, 0.0, 3.0)

        # ===== FAZA 4: GPS'SİZ DÜZ UÇUŞ =====
        self.get_logger().info('📍 FAZA 4: GPS YOK — Visual Odometry ile Düz Uçuş (10 sn)...')
        self.publish_cmd(0.5, 0.0, 10.0)

        # ===== FAZA 5: GPS'SİZ DÖNÜŞ =====
        self.get_logger().info('📍 FAZA 5: GPS YOK — VO ile Sola 90° Dönüş (4 sn)...')
        self.publish_cmd(0.0, 0.4, 4.0)

        # Dönüş sonrası kısa hover
        self.get_logger().info('📍 FAZA 5: Dönüş Sonrası Hover (2 sn)...')
        self.publish_cmd(0.0, 0.0, 2.0)

        # ===== FAZA 6: GPS'SİZ YENİ YÖN =====
        self.get_logger().info('📍 FAZA 6: GPS YOK — Yeni Yönde Uçuş (6 sn)...')
        self.publish_cmd(0.5, 0.0, 6.0)

        # ===== FAZA 7: GPS GERİ GELDİ =====
        self.get_logger().info('')
        self.get_logger().info('🟢' * 25)
        self.get_logger().info('🟢  GPS SİNYALİ GERİ GELDİ! RECOVERY BAŞLIYOR  🟢')
        self.get_logger().info('🟢' * 25)
        self.get_logger().info('')
        self.set_gps(True)

        # Recovery sırasında hover — yumuşak geçişin tamamlanmasını bekle
        self.get_logger().info('📍 FAZA 7: Yumuşak Geçiş Bekleniyor (5 sn)...')
        self.publish_cmd(0.0, 0.0, 5.0)

        # ===== FAZA 8: GPS İLE DEVAM =====
        self.get_logger().info('📍 FAZA 8: GPS Geri Geldi — Düz Uçuş (8 sn)...')
        self.publish_cmd(0.5, 0.0, 8.0)

        # ===== FAZA 9: GÖREV BİTİŞİ =====
        self.get_logger().info('📍 FAZA 9: Görev Tamamlandı. Hover Konumunda Bekliyor (3 sn)...')
        self.publish_cmd(0.0, 0.0, 3.0)

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  ✅  ARIZA SENARYO TESTİ TAMAMLANDI  ✅')
        self.get_logger().info('  Sonuçlar: ~/graduation_thesis/test_results/')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = FaultScenario()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
