#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math
import copy

class GpsVoSwitcher(Node):
    """GPS/VO Öncelik Tabanlı Anahtarlama Düğümü (Priority-Based Switching)
    
    GPS aktifken GPS odometrisini, sinyal kesildiğinde Frame-to-Map Görsel
    Odometriyi seçer. GPS geri geldiğinde lineer interpolasyon ile yumuşak
    geçiş (smooth recovery) uygular. Kalman filtresi gibi istatistiksel
    füzyon YAPILMAZ — kaynaklar arasında hakem (sensor arbitration) görevi görür.
    """
    def __init__(self):
        super().__init__('gps_vo_switcher')
        
        self.sub_gps = self.create_subscription(Odometry, '/master/gps_odom', self.gps_cb, 10)
        self.sub_vo = self.create_subscription(Odometry, '/rtabmap/vo', self.vo_cb, 10)
        self.sub_status = self.create_subscription(Bool, '/system/gps_status', self.status_cb, 10)
        
        # Sadece Odometry topic yayınla — TF YAYINLAMA!
        # TF zinciri (odom→base_link) RTAB-Map rgbd_odometry tarafından yönetiliyor.
        # Bu düğüm yalnızca konum kaynağını seçer (GPS veya VO), TF ağacına dokunmaz.
        self.pub_odom = self.create_publisher(Odometry, '/master/odom', 10)
        self.pub_mode = self.create_publisher(String, '/system/localization_mode', 10)

        self.gps_active = True
        self.last_gps_msg = None
        self.last_vo_msg = None
        
        # GPS→VO geçişinde koordinat uzayı hizalaması (coordinate frame alignment) için offset
        self.offset_x, self.offset_y, self.offset_yaw = 0.0, 0.0, 0.0
        self.offset_calculated = False

        # GPS geri geldiğinde yumuşak geçiş (smooth recovery / bumpless transfer) için
        self.recovering = False
        self.recovery_alpha = 0.0  # 0.0 = VO, 1.0 = GPS
        self.last_recovery_time = None
        self.recovery_duration = 2.0  # Tam geçiş için 2 saniye

        # Durum bilgisini yayınla
        self.mode_timer = self.create_timer(0.5, self.publish_mode)
        self.current_mode = 'GPS'

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_quat(self, yaw):
        return 0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)

    def publish_mode(self):
        """Mevcut konumlandırma modunu yayınla (metriklerin kaydedilmesi için)"""
        msg = String()
        msg.data = self.current_mode
        self.pub_mode.publish(msg)

    def status_cb(self, msg):
        if self.gps_active and not msg.data:
            self.get_logger().warn('!!! GPS SİNYALİ KOPTU (OUTAGE) !!!')
            self.current_mode = 'VO'
            self.recovering = False
            self.offset_calculated = False
        elif not self.gps_active and msg.data:
            self.get_logger().info('GPS Geri Geldi! Yumuşak geçiş başlıyor...')
            self.recovering = True
            self.recovery_alpha = 0.0
            self.last_recovery_time = self.get_clock().now()
            self.current_mode = 'RECOVERY'
        self.gps_active = msg.data

    def publish_system_state(self, odom_msg):
        """Sadece Odometry topic yayınla — TF yayınlamaz.
        TF zinciri RTAB-Map tarafından yönetilir (rgbd_odometry: odom→base_link, rtabmap: map→odom)."""
        self.pub_odom.publish(odom_msg)

    def lerp(self, a, b, t):
        """İki değer arasında lineer interpolasyon"""
        return a + (b - a) * t

    def slerp_yaw(self, yaw1, yaw2, t):
        """Açılar arası en kısa yoldan (shortest path) interpolasyon"""
        diff = math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1))
        return yaw1 + diff * t

    def vo_cb(self, msg):
        self.last_vo_msg = msg

        if not self.gps_active and not self.recovering:
            # --- GPS YOK: Tamamen Görsel Odometri (VO) ile çalış ---
            if not self.offset_calculated and self.last_gps_msg is None:
                self.get_logger().warn('GPS kesintisi başladı ama hiç referans GPS yok! VO bekleniyor...')
                return

            vo_yaw = self.get_yaw(msg.pose.pose.orientation)
            vo_x = msg.pose.pose.position.x
            vo_y = msg.pose.pose.position.y

            if not self.offset_calculated and self.last_gps_msg is not None:
                gps_yaw = self.get_yaw(self.last_gps_msg.pose.pose.orientation)
                gps_x = self.last_gps_msg.pose.pose.position.x
                gps_y = self.last_gps_msg.pose.pose.position.y
                
                # Açı (Yaw) Farkını Bul (-pi ile +pi arasına sarılmış haliyle)
                self.offset_yaw = math.atan2(math.sin(gps_yaw - vo_yaw), math.cos(gps_yaw - vo_yaw))
                
                # Rotasyon Matrisi ile X ve Y Farkını (Translation) Bul
                cos_theta = math.cos(self.offset_yaw)
                sin_theta = math.sin(self.offset_yaw)
                
                self.offset_x = gps_x - (vo_x * cos_theta - vo_y * sin_theta)
                self.offset_y = gps_y - (vo_x * sin_theta + vo_y * cos_theta)
                
                self.offset_calculated = True
                self.get_logger().info(f'Geçiş! Offset X:{self.offset_x:.2f}, Y:{self.offset_y:.2f}, Açı:{self.offset_yaw:.2f} rad')

            # Yeni Konumu Rotasyon Matrisi ile Hesapla (İç içe geçmeyi engeller!)
            cos_theta = math.cos(self.offset_yaw)
            sin_theta = math.sin(self.offset_yaw)
            
            estimated_x = (vo_x * cos_theta - vo_y * sin_theta) + self.offset_x
            estimated_y = (vo_x * sin_theta + vo_y * cos_theta) + self.offset_y
            estimated_yaw = vo_yaw + self.offset_yaw

            qx, qy, qz, qw = self.get_quat(estimated_yaw)

            out_odom = copy.deepcopy(msg)
            out_odom.pose.pose.position.x = estimated_x
            out_odom.pose.pose.position.y = estimated_y
            out_odom.pose.pose.orientation.x = qx
            out_odom.pose.pose.orientation.y = qy
            out_odom.pose.pose.orientation.z = qz
            out_odom.pose.pose.orientation.w = qw

            self.publish_system_state(out_odom)

        elif self.recovering and self.last_gps_msg is not None:
            # --- YUMUŞAK GEÇİŞ: VO konumundan GPS konumuna yavaşça yakınsa (Yüksek frekanslı vo_cb içinde) ---
            now = self.get_clock().now()
            dt = (now - self.last_recovery_time).nanoseconds / 1e9
            self.last_recovery_time = now
            
            self.recovery_alpha = min(1.0, self.recovery_alpha + (dt / self.recovery_duration))
            
            # VO'nun dönüştürülmüş konumunu hesapla (Yüksek frekanslı msg üzerinden)
            vo_yaw = self.get_yaw(msg.pose.pose.orientation)
            vo_x = msg.pose.pose.position.x
            vo_y = msg.pose.pose.position.y
            cos_theta = math.cos(self.offset_yaw)
            sin_theta = math.sin(self.offset_yaw)
            vo_aligned_x = (vo_x * cos_theta - vo_y * sin_theta) + self.offset_x
            vo_aligned_y = (vo_x * sin_theta + vo_y * cos_theta) + self.offset_y
            vo_aligned_yaw = vo_yaw + self.offset_yaw

            # GPS konumu (En son gelen düşük frekanslı GPS mesajından)
            gps_x = self.last_gps_msg.pose.pose.position.x
            gps_y = self.last_gps_msg.pose.pose.position.y
            gps_yaw = self.get_yaw(self.last_gps_msg.pose.pose.orientation)

            # Yumuşak geçiş: VO konumundan GPS konumuna lineer interpolasyon
            blended_x = self.lerp(vo_aligned_x, gps_x, self.recovery_alpha)
            blended_y = self.lerp(vo_aligned_y, gps_y, self.recovery_alpha)
            blended_yaw = self.slerp_yaw(vo_aligned_yaw, gps_yaw, self.recovery_alpha)

            qx, qy, qz, qw = self.get_quat(blended_yaw)
            
            blended_odom = copy.deepcopy(msg)
            blended_odom.pose.pose.position.x = blended_x
            blended_odom.pose.pose.position.y = blended_y
            blended_odom.pose.pose.orientation.x = qx
            blended_odom.pose.pose.orientation.y = qy
            blended_odom.pose.pose.orientation.z = qz
            blended_odom.pose.pose.orientation.w = qw
            self.publish_system_state(blended_odom)

            if self.recovery_alpha >= 1.0:
                self.recovering = False
                self.offset_calculated = False
                self.current_mode = 'GPS'
                self.get_logger().info('Yumuşak geçiş tamamlandı. Tam GPS moduna geçildi.')

    def gps_cb(self, msg):
        self.last_gps_msg = msg

        if self.gps_active and not self.recovering:
            # --- NORMAL GPS MODU ---
            self.publish_system_state(msg)

def main():
    rclpy.init()
    node = GpsVoSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()