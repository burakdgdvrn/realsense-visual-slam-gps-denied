#!/usr/bin/env python3
"""
Test Sonuçları Grafik Üretici — Tez Sonuçlar Bölümü İçin (Akademik Standart)

Üretilen Grafikler:
  1. XY Yörünge Haritası (GPS vs VO vs Recovery renk kodlu)
  2. ATE (Absolute Trajectory Error) Zaman Serisi
  3. RPE (Relative Pose Error) Zaman Serisi
  4. Mod Geçişleri Zaman Çizelgesi
  5. Ground Truth vs Tahmin X-Y Karşılaştırması
  6. Yaw Error Zaman Serisi
  7. İstatistik Özet Tablosu (ayrı grafik)

Kullanım:
  python3 plot_results.py                        # En son test dosyasını kullanır
  python3 plot_results.py odom_log_XXXX.csv      # Belirli dosyayı kullanır
"""
import csv
import sys
import os
import glob
import math
import matplotlib
matplotlib.use('Agg')  # GUI olmadan çalışsın
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
import numpy as np


# --- Renk Paleti ---
COLORS = {
    'GPS': '#2ecc71',       # Yeşil
    'VO': '#e74c3c',        # Kırmızı
    'RECOVERY': '#f39c12',  # Turuncu
}
BG_COLOR = '#1a1a2e'
GRID_COLOR = '#333355'
TEXT_COLOR = '#e0e0e0'


def load_odom_csv(filepath):
    """Odom CSV dosyasını oku — eski ve yeni format desteklenir"""
    data = {'timestamp': [], 'mode': [],
            'fused_x': [], 'fused_y': [], 'fused_yaw': [],
            'gps_x': [], 'gps_y': [], 'gps_yaw': [],
            'gt_x': [], 'gt_y': [], 'gt_yaw': [],
            'ate_fused': [], 'ate_gps': [],
            'yaw_error': [], 'rpe_trans': [], 'rpe_rot': []}

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in data:
                if key == 'mode':
                    data[key].append(row[key])
                elif key in row:
                    data[key].append(float(row[key]))
                else:
                    data[key].append(0.0)  # Eski CSV'lerde yaw_error/rpe yok
    return data


def load_event_csv(filepath):
    """Event CSV dosyasını oku"""
    events = []
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            events.append({
                'timestamp': float(row['timestamp']),
                'type': row['event_type'],
                'detail': row['detail']
            })
    return events


def load_stats_csv(filepath):
    """Statistics CSV dosyasını oku"""
    sections = {}
    current_section = None
    header = None

    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or not row[0].strip():
                continue
            if row[0].startswith('---'):
                current_section = row[0].strip('- ')
                header = None
                sections[current_section] = []
            elif row[0].startswith('==='):
                continue
            elif current_section and header is None:
                header = row
            elif current_section and header:
                sections[current_section].append(dict(zip(header, row)))
    return sections


def setup_style():
    """Koyu tema stil ayarları"""
    plt.rcParams.update({
        'figure.facecolor': BG_COLOR,
        'axes.facecolor': '#16213e',
        'axes.edgecolor': GRID_COLOR,
        'axes.labelcolor': TEXT_COLOR,
        'text.color': TEXT_COLOR,
        'xtick.color': TEXT_COLOR,
        'ytick.color': TEXT_COLOR,
        'grid.color': GRID_COLOR,
        'grid.alpha': 0.3,
        'font.size': 11,
        'axes.titlesize': 14,
        'axes.titleweight': 'bold',
        'figure.titlesize': 16,
        'figure.titleweight': 'bold',
    })


def add_gps_loss_zones(ax, events, alpha=0.15):
    """GPS kesinti bölgelerini kırmızı şeritlerle göster"""
    gps_lost_time = None
    for event in events:
        if event['type'] == 'GPS_LOST':
            gps_lost_time = event['timestamp']
        elif event['type'] == 'GPS_RECOVERED' and gps_lost_time is not None:
            ax.axvspan(gps_lost_time, event['timestamp'],
                       color='#e74c3c', alpha=alpha, label='_nolegend_')
            gps_lost_time = None
    if gps_lost_time is not None:
        ax.axvspan(gps_lost_time, ax.get_xlim()[1],
                   color='#e74c3c', alpha=alpha, label='_nolegend_')


def plot_trajectory(ax, data):
    """Grafik 1: XY Yörünge Haritası"""
    modes = data['mode']
    fx, fy = data['fused_x'], data['fused_y']
    gx, gy = data['gt_x'], data['gt_y']

    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([fx[i] for i in idx], [fy[i] for i in idx],
                      c=color, s=8, alpha=0.7, label=f'Tahmin ({mode})', zorder=3)

    ax.plot(gx, gy, color='#ffffff', linewidth=1, alpha=0.3, linestyle='--', label='Ground Truth')

    ax.scatter(fx[0], fy[0], c='#00ff88', s=120, marker='^', zorder=5, label='Başlangıç', edgecolors='white', linewidths=1.5)
    ax.scatter(fx[-1], fy[-1], c='#ff4444', s=120, marker='s', zorder=5, label='Bitiş', edgecolors='white', linewidths=1.5)

    ax.set_xlabel('X Pozisyon (m)')
    ax.set_ylabel('Y Pozisyon (m)')
    ax.set_title('Drone Yörünge Haritası (XY)')
    ax.legend(loc='best', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.set_aspect('equal')
    ax.grid(True)


def plot_ate(ax, data, events):
    """Grafik 2: ATE (Absolute Trajectory Error) Zaman Serisi"""
    t = data['timestamp']
    err_fused = data['ate_fused']
    err_gps = data['ate_gps']
    modes = data['mode']

    ax.plot(t, err_gps, color='#2ecc71', alpha=0.3, linewidth=1, label='GPS ATE')

    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [err_fused[i] for i in idx],
                      c=color, s=4, alpha=0.8, label=f'Tahmin ATE ({mode})')

    add_gps_loss_zones(ax, events)

    # Mod bazlı ortalama çizgileri
    for mode, color in [('VO', '#e74c3c'), ('GPS', '#2ecc71')]:
        mode_errors = [err_fused[i] for i, m in enumerate(modes) if m == mode]
        if mode_errors:
            avg_err = sum(mode_errors) / len(mode_errors)
            rmse = math.sqrt(sum(e**2 for e in mode_errors) / len(mode_errors))
            ax.axhline(y=avg_err, color=color, linestyle=':', alpha=0.5,
                       label=f'{mode} Mean: {avg_err:.3f}m, RMSE: {rmse:.3f}m')

    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('ATE (m)')
    ax.set_title('Absolute Trajectory Error (ATE)')
    ax.legend(loc='upper left', fontsize=7, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def plot_rpe(ax, data, events):
    """Grafik 3: RPE (Relative Pose Error) Zaman Serisi"""
    t = data['timestamp']
    rpe_trans = data['rpe_trans']
    modes = data['mode']

    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [rpe_trans[i] for i in idx],
                      c=color, s=4, alpha=0.8, label=f'RPE ({mode})')

    add_gps_loss_zones(ax, events)

    # VO ortalaması
    vo_rpe = [rpe_trans[i] for i, m in enumerate(modes) if m == 'VO']
    if vo_rpe:
        avg = sum(vo_rpe) / len(vo_rpe)
        ax.axhline(y=avg, color='#e74c3c', linestyle=':', alpha=0.7,
                   label=f'VO Mean RPE: {avg:.4f}m')

    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('RPE Translation (m)')
    ax.set_title('Relative Pose Error (RPE) — Translation')
    ax.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def plot_mode_timeline(ax, data, events):
    """Grafik 4: Mod Geçişleri Zaman Çizelgesi"""
    t = data['timestamp']
    modes = data['mode']
    mode_map = {'GPS': 2, 'RECOVERY': 1, 'VO': 0}
    mode_vals = [mode_map.get(m, 0) for m in modes]

    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [mode_vals[i] for i in idx],
                      c=color, s=15, alpha=0.9, label=mode, marker='|')

    add_gps_loss_zones(ax, events, alpha=0.2)

    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['Visual\nOdometry', 'Recovery\n(Geçiş)', 'GPS\nAktif'])
    ax.set_xlabel('Zaman (s)')
    ax.set_title('Konumlandırma Modu Zaman Çizelgesi')
    ax.set_ylim(-0.5, 2.5)
    ax.grid(True, axis='x')

    gps_lost_count = sum(1 for e in events if e['type'] == 'GPS_LOST')
    ax.text(0.98, 0.95, f'GPS Kesinti: {gps_lost_count}x',
            transform=ax.transAxes, ha='right', va='top',
            fontsize=10, color='#e74c3c',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a2e', edgecolor='#e74c3c'))


def plot_xy_comparison(ax, data, events):
    """Grafik 5: Ground Truth vs Tahmin X-Y Karşılaştırması"""
    t = data['timestamp']
    fx, fy = data['fused_x'], data['fused_y']
    gx, gy = data['gt_x'], data['gt_y']

    ax.plot(t, gx, color='#2ecc71', linewidth=1, alpha=0.6, label='Ground Truth X')
    ax.plot(t, fx, color='#e74c3c', linewidth=1, alpha=0.8, label='Tahmin X')
    ax.plot(t, gy, color='#27ae60', linewidth=1, alpha=0.6, linestyle='--', label='Ground Truth Y')
    ax.plot(t, fy, color='#c0392b', linewidth=1, alpha=0.8, linestyle='--', label='Tahmin Y')

    add_gps_loss_zones(ax, events)

    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('Pozisyon (m)')
    ax.set_title('Ground Truth vs Tahmin Pozisyon Karşılaştırması')
    ax.legend(loc='best', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def plot_yaw_error(ax, data, events):
    """Grafik 6: Yaw Error Zaman Serisi"""
    t = data['timestamp']
    yaw_err = data['yaw_error']
    modes = data['mode']

    # Derece cinsinden göster (daha okunabilir)
    yaw_err_deg = [math.degrees(y) for y in yaw_err]

    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [yaw_err_deg[i] for i in idx],
                      c=color, s=4, alpha=0.8, label=f'Yaw Error ({mode})')

    add_gps_loss_zones(ax, events)

    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('Yaw Error (°)')
    ax.set_title('Açısal (Yaw) Hata')
    ax.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def compute_baseline_trajectory(data):
    """Baseline hesaplama: Switcher kapalıyken (GPS-only) GPS kesilince ne olur?
    Tezdeki kısıtlılıklar (IMU modellemesi olmaması) göz önüne alınarak, akademik 
    olarak en savunulabilir yöntem olan 'Sabit Hız Modeli' (First-Order Hold) kullanılmıştır.
    GPS koptuğunda drone, son bilinen hız vektörüyle düz bir çizgide ilerlediği varsayılır.
    (Uydurma ivme/drift katsayıları içermez, tamamen kinematiktir.)
    """
    baseline_x = []
    baseline_y = []
    baseline_ate = []
    
    vx, vy = 0.0, 0.0
    outage_start_time = None
    drift_start_x, drift_start_y = 0.0, 0.0

    for i in range(len(data['mode'])):
        t = data['timestamp'][i]
        
        if data['mode'][i] == 'GPS':
            # GPS aktif — normal GPS pozisyonunu kullan
            baseline_x.append(data['gps_x'][i])
            baseline_y.append(data['gps_y'][i])
            outage_start_time = None
            
            # Anlık hız tahmini (son adım üzerinden)
            if i > 0:
                dt = t - data['timestamp'][i-1]
                if dt > 0.01:
                    vx = (data['gps_x'][i] - data['gps_x'][i-1]) / dt
                    vy = (data['gps_y'][i] - data['gps_y'][i-1]) / dt
        else:
            # GPS kesik (Switcher OFF) — Sabit Hızla Ölü Kestirim (First-Order Hold)
            if outage_start_time is None:
                outage_start_time = t
                # Kesintiden önceki son geçerli konumu al
                drift_start_x = baseline_x[-1] if len(baseline_x) > 0 else 0.0
                drift_start_y = baseline_y[-1] if len(baseline_y) > 0 else 0.0
                
            dt_outage = t - outage_start_time
            
            # Sabit hız formülü: x = x0 + v*t (İvme/IMU yok)
            sim_x = drift_start_x + (vx * dt_outage)
            sim_y = drift_start_y + (vy * dt_outage)
            
            baseline_x.append(sim_x)
            baseline_y.append(sim_y)

        # Baseline ATE hesapla
        gt_x = data['gt_x'][i]
        gt_y = data['gt_y'][i]
        ate = math.sqrt((baseline_x[-1] - gt_x)**2 + (baseline_y[-1] - gt_y)**2)
        baseline_ate.append(ate)

    return baseline_x, baseline_y, baseline_ate


def plot_ablation_trajectory(ax, data, events):
    """Ablation Grafiği: Switcher ON vs OFF yörünge karşılaştırması"""
    baseline_x, baseline_y, _ = compute_baseline_trajectory(data)

    # Ground Truth
    ax.plot(data['gt_x'], data['gt_y'], color='#ffffff', linewidth=1.5,
            alpha=0.4, linestyle='--', label='Ground Truth', zorder=2)

    # Baseline (Switcher OFF) — GPS kesilince donuyor
    ax.plot(baseline_x, baseline_y, color='#ff6b6b', linewidth=2,
            alpha=0.8, label='Baseline (Switcher OFF)', zorder=3)

    # Switcher ON — fused trajectory
    ax.plot(data['fused_x'], data['fused_y'], color='#51cf66', linewidth=2,
            alpha=0.8, label='Switcher ON (GPS/VO)', zorder=4)

    # GPS kesinti bölgelerindeki donma noktalarını vurgula
    freeze_points_x = []
    freeze_points_y = []
    for i in range(1, len(data['mode'])):
        if data['mode'][i] != 'GPS' and data['mode'][i-1] == 'GPS':
            freeze_points_x.append(baseline_x[i])
            freeze_points_y.append(baseline_y[i])
    if freeze_points_x:
        ax.scatter(freeze_points_x, freeze_points_y, c='#ff6b6b', s=100,
                   marker='X', zorder=5, label='GPS Lost (Frozen)', edgecolors='white', linewidths=1.5)

    # Başlangıç ve bitiş
    ax.scatter(data['fused_x'][0], data['fused_y'][0], c='#00ff88', s=120,
               marker='^', zorder=6, label='Start', edgecolors='white', linewidths=1.5)
    ax.scatter(data['fused_x'][-1], data['fused_y'][-1], c='#ffd43b', s=120,
               marker='s', zorder=6, label='End', edgecolors='white', linewidths=1.5)

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Ablation Study: Switcher ON vs OFF — Trajectory Comparison')
    ax.legend(loc='best', fontsize=9, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.set_aspect('equal')
    ax.grid(True)


def plot_ablation_ate(ax, data, events):
    """Ablation ATE Karşılaştırması: Switcher ON vs Baseline ATE zaman serisi"""
    t = data['timestamp']
    _, _, baseline_ate = compute_baseline_trajectory(data)
    fused_ate = data['ate_fused']
    modes = data['mode']

    ax.plot(t, baseline_ate, color='#ff6b6b', linewidth=1.5, alpha=0.8,
            label='Baseline (Switcher OFF)')
    ax.plot(t, fused_ate, color='#51cf66', linewidth=1.5, alpha=0.8,
            label='Switcher ON (GPS/VO)')

    add_gps_loss_zones(ax, events, alpha=0.2)

    # VO bölgelerinde ortalama ATE karşılaştırması
    vo_idx = [i for i, m in enumerate(modes) if m in ('VO', 'RECOVERY')]
    if vo_idx:
        vo_baseline_ate = [baseline_ate[i] for i in vo_idx]
        vo_fused_ate = [fused_ate[i] for i in vo_idx]
        avg_baseline = sum(vo_baseline_ate) / len(vo_baseline_ate)
        avg_fused = sum(vo_fused_ate) / len(vo_fused_ate)
        improvement = ((avg_baseline - avg_fused) / avg_baseline) * 100 if avg_baseline > 0 else 0

        ax.axhline(y=avg_baseline, color='#ff6b6b', linestyle=':', alpha=0.6,
                   label=f'Baseline Mean (outage): {avg_baseline:.2f}m')
        ax.axhline(y=avg_fused, color='#51cf66', linestyle=':', alpha=0.6,
                   label=f'Switcher Mean (outage): {avg_fused:.2f}m')

        # İyileşme yüzdesi
        ax.annotate(f'ATE Improvement: {improvement:.1f}%\n'
                    f'Baseline: {avg_baseline:.2f}m → Switcher: {avg_fused:.2f}m',
                    xy=(0.98, 0.95), xycoords='axes fraction',
                    ha='right', va='top', fontsize=10, color='#ffd43b',
                    bbox=dict(boxstyle='round,pad=0.4', facecolor='#1a1a2e',
                              edgecolor='#ffd43b', alpha=0.9))

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('ATE (m)')
    ax.set_title('Ablation Study: ATE Comparison — Switcher ON vs OFF')
    ax.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def plot_statistics_table(ax, data, events):
    """Grafik 7: İstatistik özet tablosu"""
    modes = data['mode']
    ate_fused = data['ate_fused']
    rpe_trans = data['rpe_trans']
    yaw_err = data['yaw_error']

    def calc_stats(values):
        if not values:
            return ['-'] * 5
        n = len(values)
        mean = sum(values) / n
        rmse = math.sqrt(sum(v**2 for v in values) / n)
        std = math.sqrt(sum((v - mean)**2 for v in values) / n) if n > 1 else 0
        return [f'{min(values):.4f}', f'{max(values):.4f}',
                f'{mean:.4f}', f'{rmse:.4f}', f'{std:.4f}']

    ax.axis('off')

    # Tablo verileri
    col_labels = ['Metric', 'Mode', 'Min', 'Max', 'Mean', 'RMSE', 'Std']
    table_data = []

    for mode in ['GPS', 'VO', 'RECOVERY', 'ALL']:
        if mode == 'ALL':
            ate_vals = ate_fused
            rpe_vals = rpe_trans
            yaw_vals = yaw_err
        else:
            idx = [i for i, m in enumerate(modes) if m == mode]
            ate_vals = [ate_fused[i] for i in idx]
            rpe_vals = [rpe_trans[i] for i in idx]
            yaw_vals = [yaw_err[i] for i in idx]

        if ate_vals:
            table_data.append([f'ATE (m)', mode] + calc_stats(ate_vals))
            table_data.append([f'RPE (m)', mode] + calc_stats(rpe_vals))
            table_data.append([f'Yaw (rad)', mode] + calc_stats(yaw_vals))

    if table_data:
        table = ax.table(
            cellText=table_data, colLabels=col_labels,
            cellLoc='center', loc='center',
            colColours=['#2c3e6b'] * len(col_labels)
        )
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1.0, 1.4)

        # Stil
        for (row, col), cell in table.get_celld().items():
            cell.set_edgecolor('#444466')
            if row == 0:
                cell.set_text_props(weight='bold', color='white')
                cell.set_facecolor('#2c3e6b')
            else:
                cell.set_facecolor('#1a1a3e')
                cell.set_text_props(color='#cccccc')
                # Mod renklerini uygula
                mode_val = table_data[row-1][1]
                if mode_val in COLORS:
                    table.get_celld()[(row, 1)].set_text_props(color=COLORS[mode_val])

    ax.set_title('Akademik Metrik Özet Tablosu', fontsize=14, fontweight='bold', pad=20)


def generate_summary_text(data, events):
    """Grafik altı özet metni"""
    modes = data['mode']
    errors = data['ate_fused']
    rpe = data['rpe_trans']
    total = len(modes)

    gps_count = modes.count('GPS')
    vo_count = modes.count('VO')
    rec_count = modes.count('RECOVERY')

    vo_errors = [errors[i] for i, m in enumerate(modes) if m == 'VO']
    avg_ate = sum(vo_errors) / len(vo_errors) if vo_errors else 0
    rmse_ate = math.sqrt(sum(e**2 for e in vo_errors) / len(vo_errors)) if vo_errors else 0
    max_ate = max(vo_errors) if vo_errors else 0

    vo_rpe = [rpe[i] for i, m in enumerate(modes) if m == 'VO']
    avg_rpe = sum(vo_rpe) / len(vo_rpe) if vo_rpe else 0

    gps_lost = sum(1 for e in events if e['type'] == 'GPS_LOST')

    return (f'Toplam: {total} kayıt | GPS: {gps_count} ({100*gps_count/total:.0f}%) | '
            f'VO: {vo_count} ({100*vo_count/total:.0f}%) | Recovery: {rec_count} ({100*rec_count/total:.0f}%)\n'
            f'GPS Kesinti: {gps_lost}x | VO ATE — Mean: {avg_ate:.3f}m, RMSE: {rmse_ate:.3f}m, Max: {max_ate:.3f}m | '
            f'VO RPE Mean: {avg_rpe:.4f}m')


def main():
    results_dir = os.path.expanduser('~/graduation_thesis/test_results')

    if len(sys.argv) > 1:
        odom_file = os.path.join(results_dir, sys.argv[1])
    else:
        odom_files = sorted(glob.glob(os.path.join(results_dir, 'odom_log_*.csv')))
        if not odom_files:
            print('❌ test_results/ klasöründe odom_log dosyası bulunamadı!')
            return
        odom_file = odom_files[-1]

    timestamp = os.path.basename(odom_file).replace('odom_log_', '').replace('.csv', '')
    event_file = os.path.join(results_dir, f'event_log_{timestamp}.csv')
    stats_file = os.path.join(results_dir, f'statistics_{timestamp}.csv')

    print(f'📊 Odom dosyası : {odom_file}')
    print(f'📋 Event dosyası: {event_file}')
    if os.path.exists(stats_file):
        print(f'📈 Stats dosyası: {stats_file}')

    data = load_odom_csv(odom_file)
    events = load_event_csv(event_file) if os.path.exists(event_file) else []

    print(f'📈 {len(data["timestamp"])} kayıt yüklendi, grafikler üretiliyor...')

    setup_style()

    # --- ANA FİGÜR: 6 grafik + 1 tablo ---
    fig = plt.figure(figsize=(20, 24))
    fig.suptitle('GPS/VO Anahtarlamalı Lokalizasyon — Akademik Test Sonuçları', y=0.98, fontsize=18)

    # 3x2 grid + alt kısımda tablo
    gs = fig.add_gridspec(4, 2, hspace=0.35, wspace=0.3,
                          top=0.95, bottom=0.05, left=0.06, right=0.97)

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])
    ax5 = fig.add_subplot(gs[2, 0])
    ax6 = fig.add_subplot(gs[2, 1])
    ax7 = fig.add_subplot(gs[3, :])

    plot_trajectory(ax1, data)
    plot_ate(ax2, data, events)
    plot_rpe(ax3, data, events)
    plot_mode_timeline(ax4, data, events)
    plot_xy_comparison(ax5, data, events)
    plot_yaw_error(ax6, data, events)
    plot_statistics_table(ax7, data, events)

    # Özet metni
    summary = generate_summary_text(data, events)
    fig.text(0.5, 0.01, summary, ha='center', va='bottom',
             fontsize=10, color='#aaaaaa',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f0f23', edgecolor=GRID_COLOR))

    output_path = os.path.join(results_dir, f'test_results_{timestamp}.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✅ Ana grafik kaydedildi: {output_path}')

    # --- ABLATION GRAFİKLERİ (tezin ana figürü) ---
    has_vo = any(m in ('VO', 'RECOVERY') for m in data['mode'])
    if has_vo:
        # Ablation Figür: Trajectory + ATE yan yana
        fig_abl, (ax_abl1, ax_abl2) = plt.subplots(1, 2, figsize=(22, 9))
        setup_style()
        fig_abl.suptitle('Ablation Study: GPS/VO Switcher ON vs OFF', fontsize=16, y=0.98)
        plot_ablation_trajectory(ax_abl1, data, events)
        plot_ablation_ate(ax_abl2, data, events)

        abl_path = os.path.join(results_dir, f'ablation_{timestamp}.png')
        fig_abl.savefig(abl_path, dpi=200, bbox_inches='tight')
        print(f'✅ Ablation grafiği kaydedildi: {abl_path}')

        # Ayrı ayrı da kaydet (tez için)
        fig_abl_t, ax_abl_t = plt.subplots(figsize=(12, 9))
        setup_style()
        plot_ablation_trajectory(ax_abl_t, data, events)
        fig_abl_t.savefig(os.path.join(results_dir, f'ablation_trajectory_{timestamp}.png'), dpi=200, bbox_inches='tight')

        fig_abl_a, ax_abl_a = plt.subplots(figsize=(14, 6))
        setup_style()
        plot_ablation_ate(ax_abl_a, data, events)
        fig_abl_a.savefig(os.path.join(results_dir, f'ablation_ate_{timestamp}.png'), dpi=200, bbox_inches='tight')

        print(f'✅ Ayrı ablation grafikleri kaydedildi:')
        print(f'   ablation_trajectory_{timestamp}.png')
        print(f'   ablation_ate_{timestamp}.png')
    else:
        print(f'⚠️  Bu run\'da VO/RECOVERY modu yok — ablation grafiği üretilmedi.')

    # --- AYRI BÜYÜK GRAFİKLER (tez için yüksek çözünürlük) ---
    separate_plots = [
        ('trajectory', plot_trajectory, (10, 8), None),
        ('ate', plot_ate, (12, 5), events),
        ('rpe', plot_rpe, (12, 5), events),
        ('mode_timeline', plot_mode_timeline, (12, 4), events),
        ('xy_comparison', plot_xy_comparison, (12, 5), events),
        ('yaw_error', plot_yaw_error, (12, 5), events),
    ]

    for name, plot_func, figsize, ev in separate_plots:
        fig_s, ax_s = plt.subplots(figsize=figsize)
        setup_style()
        if ev is not None:
            plot_func(ax_s, data, ev)
        else:
            plot_func(ax_s, data)
        out = os.path.join(results_dir, f'{name}_{timestamp}.png')
        fig_s.savefig(out, dpi=200, bbox_inches='tight')

    # İstatistik tablosu ayrı
    fig_t, ax_t = plt.subplots(figsize=(14, 8))
    setup_style()
    plot_statistics_table(ax_t, data, events)
    fig_t.savefig(os.path.join(results_dir, f'statistics_table_{timestamp}.png'), dpi=200, bbox_inches='tight')

    plt.close('all')

    print(f'✅ Ayrı grafikler kaydedildi:')
    for name, _, _, _ in separate_plots:
        print(f'   {name}_{timestamp}.png')
    print(f'   statistics_table_{timestamp}.png')
    print(f'\n📁 Tüm çıktılar: {results_dir}/')


if __name__ == '__main__':
    main()
