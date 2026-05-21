#!/usr/bin/env python3
"""
Metrik Kayıt Sistemi — Tez Sonuçları İçin Veri Toplayıcı (Akademik Standart)

Kaydedilen Metrikler:
  - ATE (Absolute Trajectory Error): Füzyon konumu vs Ground Truth Öklid mesafesi
  - RPE (Relative Pose Error): Ardışık frameler arası pose değişim hatası
  - GPS ATE: Gürültülü GPS vs Ground Truth Öklid mesafesi
  - Yaw Hatası: Açısal sapma (rad)
  - Mod geçiş zamanlamaları (GPS → VO → RECOVERY → GPS)

Çıktı:
  - ~/graduation_thesis/test_results/odom_log_*.csv     — Ham veri
  - ~/graduation_thesis/test_results/event_log_*.csv    — Olay kaydı
  - ~/graduation_thesis/test_results/statistics_*.csv   — İstatistik özet tablosu
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
        self.timestamp = timestamp

        self.odom_csv_path = os.path.join(self.results_dir, f'odom_log_{timestamp}.csv')
        self.event_csv_path = os.path.join(self.results_dir, f'event_log_{timestamp}.csv')
        self.stats_csv_path = os.path.join(self.results_dir, f'statistics_{timestamp}.csv')

        # --- Odometri CSV ---
        self.odom_file = open(self.odom_csv_path, 'w', newline='')
        self.odom_writer = csv.writer(self.odom_file)
        self.odom_writer.writerow([
            'timestamp', 'mode',
            'fused_x', 'fused_y', 'fused_yaw',
            'gps_x', 'gps_y', 'gps_yaw',
            'gt_x', 'gt_y', 'gt_yaw',
            'ate_fused', 'ate_gps',
            'yaw_error',
            'rpe_trans', 'rpe_rot'
        ])

        # --- Olay (Event) CSV ---
        self.event_file = open(self.event_csv_path, 'w', newline='')
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(['timestamp', 'event_type', 'detail'])

        # --- Durum Değişkenleri ---
        self.current_mode = 'GPS'
        self.last_mode = 'GPS'
        self.gps_active = True
        self.last_gps_odom = None
        self.last_fused_odom = None
        self.record_count = 0

        # Ground truth pozisyonu
        self.master_pose = None

        # RPE hesabı için önceki frame verileri
        self.prev_fused_x = None
        self.prev_fused_y = None
        self.prev_fused_yaw = None
        self.prev_gt_x = None
        self.prev_gt_y = None
        self.prev_gt_yaw = None

        # İstatistik toplama (shutdown'da özet yazmak için)
        self.all_ate_fused = []
        self.all_ate_gps = []
        self.all_yaw_error = []
        self.all_rpe_trans = []
        self.all_rpe_rot = []
        # Mod bazlı ayrıştırma
        self.mode_ate_fused = {'GPS': [], 'VO': [], 'RECOVERY': []}
        self.mode_rpe_trans = {'GPS': [], 'VO': [], 'RECOVERY': []}

        # --- Subscriber'lar ---
        self.create_subscription(Odometry, '/master/odom', self.fused_odom_cb, 10)
        self.create_subscription(Odometry, '/master/gps_odom', self.gps_odom_cb, 10)
        self.create_subscription(String, '/system/localization_mode', self.mode_cb, 10)
        self.create_subscription(Bool, '/system/gps_status', self.gps_status_cb, 10)
        self.create_subscription(Pose, '/swarm/master_target', self.master_pose_cb, 10)

        # --- Timer: Her 0.5 saniyede kayıt ---
        self.create_timer(0.5, self.record_metrics)

        self.get_logger().info(f'📊 Metrik Kaydı Başladı!')
        self.get_logger().info(f'   Odom Log  : {self.odom_csv_path}')
        self.get_logger().info(f'   Event Log : {self.event_csv_path}')
        self.get_logger().info(f'   Stats     : {self.stats_csv_path}')

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        """İki açı arasındaki en kısa fark (wrap -pi/+pi)"""
        return math.atan2(math.sin(a - b), math.cos(a - b))

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
            self.event_writer.writerow([f'{now:.3f}', 'GPS_LOST', 'GPS signal lost'])
            self.event_file.flush()
        elif not self.gps_active and msg.data:
            self.event_writer.writerow([f'{now:.3f}', 'GPS_RECOVERED', 'GPS signal restored'])
            self.event_file.flush()
        self.gps_active = msg.data

    def master_pose_cb(self, msg):
        self.master_pose = msg

    # --- RPE Hesabı ---
    def compute_rpe(self, fx, fy, fyaw, gt_x, gt_y, gt_yaw):
        """
        RPE (Relative Pose Error): Ardışık iki frame arasındaki
        ground truth hareket ile tahmin edilen hareket arasındaki fark.
        
        rpe_trans = |delta_estimated - delta_gt| (Öklid)
        rpe_rot = |delta_yaw_estimated - delta_yaw_gt| (açısal)
        """
        if self.prev_fused_x is None:
            # İlk frame, RPE hesaplanamaz
            self.prev_fused_x = fx
            self.prev_fused_y = fy
            self.prev_fused_yaw = fyaw
            self.prev_gt_x = gt_x
            self.prev_gt_y = gt_y
            self.prev_gt_yaw = gt_yaw
            return 0.0, 0.0

        # Tahmin edilen hareket (fused)
        delta_fx = fx - self.prev_fused_x
        delta_fy = fy - self.prev_fused_y
        delta_fused_dist = math.sqrt(delta_fx**2 + delta_fy**2)
        delta_fused_yaw = self.angle_diff(fyaw, self.prev_fused_yaw)

        # Ground truth hareket
        delta_gx = gt_x - self.prev_gt_x
        delta_gy = gt_y - self.prev_gt_y
        delta_gt_dist = math.sqrt(delta_gx**2 + delta_gy**2)
        delta_gt_yaw = self.angle_diff(gt_yaw, self.prev_gt_yaw)

        # RPE = fark
        rpe_trans = abs(delta_fused_dist - delta_gt_dist)
        rpe_rot = abs(self.angle_diff(delta_fused_yaw, delta_gt_yaw))

        # Güncelle
        self.prev_fused_x = fx
        self.prev_fused_y = fy
        self.prev_fused_yaw = fyaw
        self.prev_gt_x = gt_x
        self.prev_gt_y = gt_y
        self.prev_gt_yaw = gt_yaw

        return rpe_trans, rpe_rot

    # --- Periyodik Kayıt ---
    def record_metrics(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.last_fused_odom is not None:
            fp = self.last_fused_odom.pose.pose.position
            fo = self.last_fused_odom.pose.pose.orientation
            fused_yaw = self.get_yaw(fo)

            gps_x, gps_y, gps_yaw = 0.0, 0.0, 0.0
            gt_x, gt_y, gt_yaw = 0.0, 0.0, 0.0
            ate_fused, ate_gps = 0.0, 0.0
            yaw_error = 0.0
            rpe_trans, rpe_rot = 0.0, 0.0

            if self.master_pose is not None:
                gt_x = self.master_pose.position.x
                gt_y = self.master_pose.position.y
                gt_yaw = self.get_yaw(self.master_pose.orientation)
                ate_fused = math.sqrt((fp.x - gt_x)**2 + (fp.y - gt_y)**2)
                yaw_error = abs(self.angle_diff(fused_yaw, gt_yaw))

                # RPE hesabı
                rpe_trans, rpe_rot = self.compute_rpe(
                    fp.x, fp.y, fused_yaw, gt_x, gt_y, gt_yaw
                )

            if self.last_gps_odom is not None:
                gp = self.last_gps_odom.pose.pose.position
                go = self.last_gps_odom.pose.pose.orientation
                gps_x, gps_y = gp.x, gp.y
                gps_yaw = self.get_yaw(go)
                if self.master_pose is not None:
                    ate_gps = math.sqrt((gps_x - gt_x)**2 + (gps_y - gt_y)**2)

            self.odom_writer.writerow([
                f'{now:.3f}', self.current_mode,
                f'{fp.x:.4f}', f'{fp.y:.4f}', f'{fused_yaw:.4f}',
                f'{gps_x:.4f}', f'{gps_y:.4f}', f'{gps_yaw:.4f}',
                f'{gt_x:.4f}', f'{gt_y:.4f}', f'{gt_yaw:.4f}',
                f'{ate_fused:.4f}', f'{ate_gps:.4f}',
                f'{yaw_error:.4f}',
                f'{rpe_trans:.4f}', f'{rpe_rot:.4f}'
            ])
            self.odom_file.flush()
            self.record_count += 1

            # İstatistik toplama
            self.all_ate_fused.append(ate_fused)
            self.all_ate_gps.append(ate_gps)
            self.all_yaw_error.append(yaw_error)
            self.all_rpe_trans.append(rpe_trans)
            self.all_rpe_rot.append(rpe_rot)

            mode = self.current_mode
            if mode in self.mode_ate_fused:
                self.mode_ate_fused[mode].append(ate_fused)
                self.mode_rpe_trans[mode].append(rpe_trans)

        # Her 20 kayıtta durum raporu
        if self.record_count > 0 and self.record_count % 20 == 0:
            self.get_logger().info(f'📊 {self.record_count} kayıt yazıldı | Mod: {self.current_mode}')

    # --- İstatistik Fonksiyonları ---
    def _calc_stats(self, values):
        """Min, Max, Mean, Median, RMSE, Std hesapla"""
        if not values:
            return {'min': 0, 'max': 0, 'mean': 0, 'median': 0, 'rmse': 0, 'std': 0, 'count': 0}

        n = len(values)
        sorted_vals = sorted(values)
        mean = sum(values) / n
        median = sorted_vals[n // 2] if n % 2 == 1 else (sorted_vals[n//2 - 1] + sorted_vals[n//2]) / 2
        rmse = math.sqrt(sum(v**2 for v in values) / n)
        variance = sum((v - mean)**2 for v in values) / n if n > 1 else 0
        std = math.sqrt(variance)

        return {
            'min': min(values),
            'max': max(values),
            'mean': mean,
            'median': median,
            'rmse': rmse,
            'std': std,
            'count': n
        }

    def write_statistics(self):
        """Shutdown'da kapsamlı istatistik özet tablosu yaz"""
        if self.record_count == 0:
            self.get_logger().warn('Hiç kayıt yapılmadı, istatistik yazılmıyor.')
            return

        with open(self.stats_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)

            # Başlık
            writer.writerow(['=== ACADEMIC METRICS SUMMARY ==='])
            writer.writerow([])

            # 1. Genel İstatistikler
            writer.writerow(['--- Overall Statistics ---'])
            writer.writerow(['Metric', 'Count', 'Min', 'Max', 'Mean', 'Median', 'RMSE', 'Std'])

            for name, values in [
                ('ATE_Fused (m)', self.all_ate_fused),
                ('ATE_GPS (m)', self.all_ate_gps),
                ('Yaw_Error (rad)', self.all_yaw_error),
                ('RPE_Trans (m)', self.all_rpe_trans),
                ('RPE_Rot (rad)', self.all_rpe_rot),
            ]:
                s = self._calc_stats(values)
                writer.writerow([
                    name, s['count'],
                    f'{s["min"]:.4f}', f'{s["max"]:.4f}',
                    f'{s["mean"]:.4f}', f'{s["median"]:.4f}',
                    f'{s["rmse"]:.4f}', f'{s["std"]:.4f}'
                ])

            writer.writerow([])

            # 2. Mod Bazlı ATE İstatistikleri
            writer.writerow(['--- Per-Mode ATE Statistics ---'])
            writer.writerow(['Mode', 'Count', 'Min', 'Max', 'Mean', 'Median', 'RMSE', 'Std'])
            for mode in ['GPS', 'VO', 'RECOVERY']:
                values = self.mode_ate_fused.get(mode, [])
                if values:
                    s = self._calc_stats(values)
                    writer.writerow([
                        mode, s['count'],
                        f'{s["min"]:.4f}', f'{s["max"]:.4f}',
                        f'{s["mean"]:.4f}', f'{s["median"]:.4f}',
                        f'{s["rmse"]:.4f}', f'{s["std"]:.4f}'
                    ])

            writer.writerow([])

            # 3. Mod Bazlı RPE İstatistikleri
            writer.writerow(['--- Per-Mode RPE Statistics ---'])
            writer.writerow(['Mode', 'Count', 'Min', 'Max', 'Mean', 'Median', 'RMSE', 'Std'])
            for mode in ['GPS', 'VO', 'RECOVERY']:
                values = self.mode_rpe_trans.get(mode, [])
                if values:
                    s = self._calc_stats(values)
                    writer.writerow([
                        mode, s['count'],
                        f'{s["min"]:.4f}', f'{s["max"]:.4f}',
                        f'{s["mean"]:.4f}', f'{s["median"]:.4f}',
                        f'{s["rmse"]:.4f}', f'{s["std"]:.4f}'
                    ])

            writer.writerow([])

            # 4. Mod Dağılımı
            writer.writerow(['--- Mode Distribution ---'])
            writer.writerow(['Mode', 'Samples', 'Percentage'])
            total = self.record_count
            for mode in ['GPS', 'VO', 'RECOVERY']:
                count = len(self.mode_ate_fused.get(mode, []))
                pct = (count / total * 100) if total > 0 else 0
                writer.writerow([mode, count, f'{pct:.1f}%'])

        self.get_logger().info(f'📊 İstatistik tablosu yazıldı: {self.stats_csv_path}')

    def destroy_node(self):
        self.get_logger().info(f'📊 Kayıt Tamamlandı! Toplam {self.record_count} odometri kaydı.')
        self.write_statistics()
        self.odom_file.close()
        self.event_file.close()
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
