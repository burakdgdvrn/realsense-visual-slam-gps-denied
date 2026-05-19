#!/usr/bin/env python3
"""
Metrik Kayıt Sistemi — Tez Sonuçları İçin Veri Toplayıcı

Kaydedilen Metrikler:
  - GPS konum hatası (VO konumu vs GPS ground truth)
  - Formasyon hatası (Slave'lerin hedef pozisyona Öklid mesafesi)
  - Yörünge sapması (Konum değişim grafiği)
  - Mod geçiş zamanlamaları (GPS → VO → RECOVERY → GPS)

Çıktı: ~/graduation_thesis/test_results/ altına zaman damgalı CSV dosyaları
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
import math
import csv
import os
from datetime import datetime


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__('metrics_recorder')

        # --- CSV Dosya Yolu ---
        self.results_dir = os.path.expanduser('~/graduation_thesis/test_results')
        os.makedirs(self.results_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.odom_csv_path = os.path.join(self.results_dir, f'odom_log_{timestamp}.csv')
        self.event_csv_path = os.path.join(self.results_dir, f'event_log_{timestamp}.csv')
        self.formation_csv_path = os.path.join(self.results_dir, f'formation_log_{timestamp}.csv')

        # --- Odometri CSV ---
        self.odom_file = open(self.odom_csv_path, 'w', newline='')
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow([
            'timestamp', 'mode',
            'fused_x', 'fused_y', 'fused_yaw',
            'gps_x', 'gps_y', 'gps_yaw',
            'position_error'
        ])

        # --- Olay (Event) CSV ---
        self.event_file = open(self.event_csv_path, 'w', newline='')
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(['timestamp', 'event_type', 'detail'])

        # --- Formasyon CSV ---
        self.formation_file = open(self.formation_csv_path, 'w', newline='')
        self.formation_writer = csv.writer(self.formation_file)
        self.formation_writer.writerow([
            'timestamp',
            'master_x', 'master_y',
            'slave1_x', 'slave1_y', 'slave1_error',
            'slave2_x', 'slave2_y', 'slave2_error'
        ])

        # --- Durum Değişkenleri ---
        self.current_mode = 'GPS'
        self.last_mode = 'GPS'
        self.gps_active = True
        self.last_gps_odom = None
        self.last_fused_odom = None
        self.record_count = 0

        # Formasyon pozisyonları
        self.master_pose = None
        self.slave1_pose = None
        self.slave2_pose = None

        # --- Subscriber'lar ---
        # Füzyon çıktısı (HybridLocalizer'dan)
        self.create_subscription(Odometry, '/master/odom', self.fused_odom_cb, 10)
        # Ham GPS verisi
        self.create_subscription(Odometry, '/master/gps_odom', self.gps_odom_cb, 10)
        # Mod bilgisi
        self.create_subscription(String, '/system/localization_mode', self.mode_cb, 10)
        # GPS durumu
        self.create_subscription(Bool, '/system/gps_status', self.gps_status_cb, 10)
        # Formasyon pozisyonları
        self.create_subscription(Pose, '/swarm/master_target', self.master_pose_cb, 10)
        self.create_subscription(Pose, '/swarm/slave1_target', self.slave1_pose_cb, 10)
        self.create_subscription(Pose, '/swarm/slave2_target', self.slave2_pose_cb, 10)

        # --- Timer: Her 0.5 saniyede kayıt ---
        self.create_timer(0.5, self.record_metrics)

        self.get_logger().info(f'📊 Metrik Kaydı Başladı!')
        self.get_logger().info(f'   Odom Log  : {self.odom_csv_path}')
        self.get_logger().info(f'   Event Log : {self.event_csv_path}')
        self.get_logger().info(f'   Formation : {self.formation_csv_path}')

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # --- Callback'ler ---
    def fused_odom_cb(self, msg):
        self.last_fused_odom = msg

    def gps_odom_cb(self, msg):
        self.last_gps_odom = msg

    def mode_cb(self, msg):
        new_mode = msg.data
        if new_mode != self.last_mode:
            now = self.get_clock().now().nanoseconds / 1e9
            self.event_writer.writerow([
                f'{now:.3f}', 'MODE_CHANGE', f'{self.last_mode} -> {new_mode}'
            ])
            self.event_file.flush()
            self.get_logger().info(f'📝 Mod Değişimi: {self.last_mode} → {new_mode}')
            self.last_mode = new_mode
        self.current_mode = new_mode

    def gps_status_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.gps_active and not msg.data:
            self.event_writer.writerow([f'{now:.3f}', 'GPS_LOST', 'GPS signal jammed'])
            self.event_file.flush()
        elif not self.gps_active and msg.data:
            self.event_writer.writerow([f'{now:.3f}', 'GPS_RECOVERED', 'GPS signal restored'])
            self.event_file.flush()
        self.gps_active = msg.data

    def master_pose_cb(self, msg):
        self.master_pose = msg

    def slave1_pose_cb(self, msg):
        self.slave1_pose = msg

    def slave2_pose_cb(self, msg):
        self.slave2_pose = msg

    # --- Periyodik Kayıt ---
    def record_metrics(self):
        now = self.get_clock().now().nanoseconds / 1e9

        # 1. Odometri kaydı
        if self.last_fused_odom is not None:
            fp = self.last_fused_odom.pose.pose.position
            fo = self.last_fused_odom.pose.pose.orientation
            fused_yaw = self.get_yaw(fo)

            gps_x, gps_y, gps_yaw = 0.0, 0.0, 0.0
            pos_error = 0.0

            if self.last_gps_odom is not None:
                gp = self.last_gps_odom.pose.pose.position
                go = self.last_gps_odom.pose.pose.orientation
                gps_x, gps_y = gp.x, gp.y
                gps_yaw = self.get_yaw(go)
                pos_error = math.sqrt((fp.x - gps_x)**2 + (fp.y - gps_y)**2)

            self.odom_writer.writerow([
                f'{now:.3f}', self.current_mode,
                f'{fp.x:.4f}', f'{fp.y:.4f}', f'{fused_yaw:.4f}',
                f'{gps_x:.4f}', f'{gps_y:.4f}', f'{gps_yaw:.4f}',
                f'{pos_error:.4f}'
            ])
            self.odom_file.flush()
            self.record_count += 1

        # 2. Formasyon kaydı
        if self.master_pose is not None:
            m = self.master_pose.position
            s1_x, s1_y, s1_err = 0.0, 0.0, 0.0
            s2_x, s2_y, s2_err = 0.0, 0.0, 0.0

            master_yaw = self.master_pose.orientation.w

            if self.slave1_pose is not None:
                s1 = self.slave1_pose.position
                s1_x, s1_y = s1.x, s1.y
                # Beklenen slave1 pozisyonu (formasyon matematiği)
                exp_s1_x = m.x - 1.0 * math.cos(master_yaw) - 1.0 * math.sin(master_yaw)
                exp_s1_y = m.y - 1.0 * math.sin(master_yaw) + 1.0 * math.cos(master_yaw)
                s1_err = math.sqrt((s1_x - exp_s1_x)**2 + (s1_y - exp_s1_y)**2)

            if self.slave2_pose is not None:
                s2 = self.slave2_pose.position
                s2_x, s2_y = s2.x, s2.y
                exp_s2_x = m.x - 1.0 * math.cos(master_yaw) + 1.0 * math.sin(master_yaw)
                exp_s2_y = m.y - 1.0 * math.sin(master_yaw) - 1.0 * math.cos(master_yaw)
                s2_err = math.sqrt((s2_x - exp_s2_x)**2 + (s2_y - exp_s2_y)**2)

            self.formation_writer.writerow([
                f'{now:.3f}',
                f'{m.x:.4f}', f'{m.y:.4f}',
                f'{s1_x:.4f}', f'{s1_y:.4f}', f'{s1_err:.4f}',
                f'{s2_x:.4f}', f'{s2_y:.4f}', f'{s2_err:.4f}'
            ])
            self.formation_file.flush()

        # Her 20 kayıtta durum raporu
        if self.record_count > 0 and self.record_count % 20 == 0:
            self.get_logger().info(f'📊 {self.record_count} kayıt yazıldı | Mod: {self.current_mode}')

    def destroy_node(self):
        self.get_logger().info(f'📊 Kayıt Tamamlandı! Toplam {self.record_count} odometri kaydı.')
        self.odom_file.close()
        self.event_file.close()
        self.formation_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
