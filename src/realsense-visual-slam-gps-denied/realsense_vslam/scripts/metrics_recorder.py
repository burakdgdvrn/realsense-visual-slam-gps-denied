#!/usr/bin/env python3
"""
Metrik Kayıt Sistemi — Tez Sonuçları İçin Veri Toplayıcı

Kaydedilen Metrikler:
  - Zaman damgası (saniye)
  - Konumlandırma modu (GPS / VO / RECOVERY)
  - Fused konum (x, y) — HybridLocalizer çıkışı
  - GPS ground truth konum (x, y)
  - Konum hatası (GPS vs Fused arasındaki Öklid mesafesi)
  - Yaw açısı (derece)

Çıktı: ~/graduation_thesis/test_results/ altına zaman damgalı CSV dosyası
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import csv
import os
from datetime import datetime


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')

        # --- Abonelikler ---
        self.sub_fused = self.create_subscription(
            Odometry, '/master/odom', self.fused_cb, 10)
        self.sub_gps = self.create_subscription(
            Odometry, '/master/gps_odom', self.gps_cb, 10)
        self.sub_mode = self.create_subscription(
            String, '/system/localization_mode', self.mode_cb, 10)

        # --- Durum Değişkenleri ---
        self.current_mode = 'GPS'
        self.fused_x, self.fused_y, self.fused_yaw = 0.0, 0.0, 0.0
        self.gps_x, self.gps_y, self.gps_yaw = 0.0, 0.0, 0.0
        self.start_time = self.get_clock().now()

        # --- CSV Dosyası Oluştur ---
        results_dir = os.path.expanduser('~/graduation_thesis/test_results')
        os.makedirs(results_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(results_dir, f'metrics_{timestamp}.csv')

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time_sec',
            'mode',
            'fused_x', 'fused_y', 'fused_yaw_deg',
            'gps_x', 'gps_y', 'gps_yaw_deg',
            'position_error_m'
        ])

        # --- 10 Hz ile kayıt ---
        self.timer = self.create_timer(0.1, self.record_metrics)
        self.record_count = 0

        self.get_logger().info(f'📊 Metrik Kaydı Başladı → {self.csv_path}')

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def mode_cb(self, msg):
        self.current_mode = msg.data

    def fused_cb(self, msg):
        self.fused_x = msg.pose.pose.position.x
        self.fused_y = msg.pose.pose.position.y
        self.fused_yaw = self.get_yaw(msg.pose.pose.orientation)

    def gps_cb(self, msg):
        self.gps_x = msg.pose.pose.position.x
        self.gps_y = msg.pose.pose.position.y
        self.gps_yaw = self.get_yaw(msg.pose.pose.orientation)

    def record_metrics(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Öklid mesafesi: fused konum vs GPS ground truth
        error = math.sqrt(
            (self.fused_x - self.gps_x) ** 2 +
            (self.fused_y - self.gps_y) ** 2
        )

        self.csv_writer.writerow([
            f'{elapsed:.2f}',
            self.current_mode,
            f'{self.fused_x:.4f}', f'{self.fused_y:.4f}',
            f'{math.degrees(self.fused_yaw):.2f}',
            f'{self.gps_x:.4f}', f'{self.gps_y:.4f}',
            f'{math.degrees(self.gps_yaw):.2f}',
            f'{error:.4f}'
        ])
        self.csv_file.flush()

        self.record_count += 1
        # Her 50 kayıtta bir durum bilgisi (5 saniyede bir)
        if self.record_count % 50 == 0:
            self.get_logger().info(
                f'[{self.current_mode:>8}] Konum: ({self.fused_x:.2f}, {self.fused_y:.2f}) '
                f'| GPS: ({self.gps_x:.2f}, {self.gps_y:.2f}) '
                f'| Hata: {error:.3f}m '
                f'| Kayıt: {self.record_count}'
            )

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(
            f'📊 Metrik Kaydı Tamamlandı! {self.record_count} satır → {self.csv_path}'
        )
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
