#!/usr/bin/env python3
"""
Test Sonuçları Grafik Üretici — Tez Sonuçlar Bölümü İçin

Üretilen Grafikler:
  1. XY Yörünge Haritası (GPS vs VO vs Recovery renk kodlu)
  2. Konum Hatası Zaman Serisi (GPS kesinti bölgeleri vurgulanmış)
  3. Mod Geçişleri Zaman Çizelgesi
  4. GPS vs Füzyon Karşılaştırma (X ve Y ayrı ayrı)

Kullanım:
  python3 plot_results.py                        # En son test dosyasını kullanır
  python3 plot_results.py odom_log_XXXX.csv      # Belirli dosyayı kullanır
"""
import csv
import sys
import os
import glob
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
    """Odom CSV dosyasını oku"""
    data = {'timestamp': [], 'mode': [],
            'fused_x': [], 'fused_y': [], 'fused_yaw': [],
            'gps_x': [], 'gps_y': [], 'gps_yaw': [],
            'position_error': []}
    
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in data:
                if key == 'mode':
                    data[key].append(row[key])
                else:
                    data[key].append(float(row[key]))
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
    # Son kesinti devam ediyorsa
    if gps_lost_time is not None:
        ax.axvspan(gps_lost_time, ax.get_xlim()[1],
                   color='#e74c3c', alpha=alpha, label='_nolegend_')


def plot_trajectory(ax, data):
    """Grafik 1: XY Yörünge Haritası"""
    modes = data['mode']
    fx, fy = data['fused_x'], data['fused_y']
    gx, gy = data['gps_x'], data['gps_y']
    
    # Her mod için ayrı scatter
    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([fx[i] for i in idx], [fy[i] for i in idx],
                      c=color, s=8, alpha=0.7, label=f'Füzyon ({mode})', zorder=3)
    
    # GPS ground truth çizgisi
    ax.plot(gx, gy, color='#ffffff', linewidth=1, alpha=0.3, linestyle='--', label='GPS Ground Truth')
    
    # Başlangıç ve bitiş noktaları
    ax.scatter(fx[0], fy[0], c='#00ff88', s=120, marker='^', zorder=5, label='Başlangıç', edgecolors='white', linewidths=1.5)
    ax.scatter(fx[-1], fy[-1], c='#ff4444', s=120, marker='s', zorder=5, label='Bitiş', edgecolors='white', linewidths=1.5)
    
    ax.set_xlabel('X Pozisyon (m)')
    ax.set_ylabel('Y Pozisyon (m)')
    ax.set_title('Drone Yörünge Haritası (XY)')
    ax.legend(loc='best', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.set_aspect('equal')
    ax.grid(True)


def plot_position_error(ax, data, events):
    """Grafik 2: Konum Hatası Zaman Serisi"""
    t = data['timestamp']
    err = data['position_error']
    modes = data['mode']
    
    # Renkli çizgi
    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [err[i] for i in idx],
                      c=color, s=4, alpha=0.8, label=mode)
    
    # GPS kesinti bölgeleri
    add_gps_loss_zones(ax, events)
    
    # Ortalama hata çizgisi (sadece VO)
    vo_errors = [err[i] for i, m in enumerate(modes) if m == 'VO']
    if vo_errors:
        avg_err = sum(vo_errors) / len(vo_errors)
        ax.axhline(y=avg_err, color='#e74c3c', linestyle=':', alpha=0.7,
                   label=f'VO Ort. Hata: {avg_err:.3f}m')
    
    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('Konum Hatası (m)')
    ax.set_title('GPS vs Füzyon Konum Hatası')
    ax.legend(loc='upper left', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def plot_mode_timeline(ax, data, events):
    """Grafik 3: Mod Geçişleri Zaman Çizelgesi"""
    t = data['timestamp']
    modes = data['mode']
    mode_map = {'GPS': 2, 'RECOVERY': 1, 'VO': 0}
    mode_vals = [mode_map.get(m, 0) for m in modes]
    
    # Renkli çubuklar
    for mode, color in COLORS.items():
        idx = [i for i, m in enumerate(modes) if m == mode]
        if idx:
            ax.scatter([t[i] for i in idx], [mode_vals[i] for i in idx],
                      c=color, s=15, alpha=0.9, label=mode, marker='|')
    
    # GPS kesinti bölgeleri
    add_gps_loss_zones(ax, events, alpha=0.2)
    
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(['Visual\nOdometry', 'Recovery\n(Geçiş)', 'GPS\nAktif'])
    ax.set_xlabel('Zaman (s)')
    ax.set_title('Konumlandırma Modu Zaman Çizelgesi')
    ax.set_ylim(-0.5, 2.5)
    ax.grid(True, axis='x')
    
    # GPS kesinti sayısını göster
    gps_lost_count = sum(1 for e in events if e['type'] == 'GPS_LOST')
    ax.text(0.98, 0.95, f'GPS Kesinti: {gps_lost_count}x',
            transform=ax.transAxes, ha='right', va='top',
            fontsize=10, color='#e74c3c',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a2e', edgecolor='#e74c3c'))


def plot_xy_comparison(ax, data, events):
    """Grafik 4: GPS vs Füzyon X-Y Karşılaştırması"""
    t = data['timestamp']
    fx, fy = data['fused_x'], data['fused_y']
    gx, gy = data['gps_x'], data['gps_y']
    
    ax.plot(t, gx, color='#2ecc71', linewidth=1, alpha=0.6, label='GPS X')
    ax.plot(t, fx, color='#e74c3c', linewidth=1, alpha=0.8, label='Füzyon X')
    ax.plot(t, gy, color='#27ae60', linewidth=1, alpha=0.6, linestyle='--', label='GPS Y')
    ax.plot(t, fy, color='#c0392b', linewidth=1, alpha=0.8, linestyle='--', label='Füzyon Y')
    
    add_gps_loss_zones(ax, events)
    
    ax.set_xlabel('Zaman (s)')
    ax.set_ylabel('Pozisyon (m)')
    ax.set_title('GPS vs Füzyon Pozisyon Karşılaştırması')
    ax.legend(loc='best', fontsize=8, facecolor='#1a1a2e', edgecolor=GRID_COLOR)
    ax.grid(True)


def generate_summary_text(data, events):
    """Grafik altı özet metni"""
    modes = data['mode']
    errors = data['position_error']
    total = len(modes)
    
    gps_count = modes.count('GPS')
    vo_count = modes.count('VO')
    rec_count = modes.count('RECOVERY')
    
    vo_errors = [errors[i] for i, m in enumerate(modes) if m == 'VO']
    avg_err = sum(vo_errors) / len(vo_errors) if vo_errors else 0
    max_err = max(vo_errors) if vo_errors else 0
    
    gps_lost = sum(1 for e in events if e['type'] == 'GPS_LOST')
    
    return (f'Toplam: {total} kayıt | GPS: {gps_count} ({100*gps_count/total:.0f}%) | '
            f'VO: {vo_count} ({100*vo_count/total:.0f}%) | Recovery: {rec_count} ({100*rec_count/total:.0f}%)\n'
            f'GPS Kesinti: {gps_lost}x | VO Ort. Hata: {avg_err:.3f}m | VO Max Hata: {max_err:.3f}m')


def main():
    results_dir = os.path.expanduser('~/graduation_thesis/test_results')
    
    # Dosya seçimi
    if len(sys.argv) > 1:
        odom_file = os.path.join(results_dir, sys.argv[1])
    else:
        # En son dosyayı bul
        odom_files = sorted(glob.glob(os.path.join(results_dir, 'odom_log_*.csv')))
        if not odom_files:
            print('❌ test_results/ klasöründe odom_log dosyası bulunamadı!')
            return
        odom_file = odom_files[-1]
    
    # Event dosyasını bul (aynı timestamp)
    timestamp = os.path.basename(odom_file).replace('odom_log_', '').replace('.csv', '')
    event_file = os.path.join(results_dir, f'event_log_{timestamp}.csv')
    
    print(f'📊 Odom dosyası : {odom_file}')
    print(f'📋 Event dosyası: {event_file}')
    
    # Verileri yükle
    data = load_odom_csv(odom_file)
    events = load_event_csv(event_file) if os.path.exists(event_file) else []
    
    print(f'📈 {len(data["timestamp"])} kayıt yüklendi, grafikler üretiliyor...')
    
    # Stil ayarla
    setup_style()
    
    # --- ANA FİGÜR: 4 grafik tek sayfada ---
    fig, axes = plt.subplots(2, 2, figsize=(18, 12))
    fig.suptitle('GPS-Denied Visual SLAM — Test Sonuçları', y=0.98)
    
    plot_trajectory(axes[0, 0], data)
    plot_position_error(axes[0, 1], data, events)
    plot_mode_timeline(axes[1, 0], data, events)
    plot_xy_comparison(axes[1, 1], data, events)
    
    # Özet metni
    summary = generate_summary_text(data, events)
    fig.text(0.5, 0.01, summary, ha='center', va='bottom',
             fontsize=10, color='#aaaaaa',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#0f0f23', edgecolor=GRID_COLOR))
    
    plt.tight_layout(rect=[0, 0.04, 1, 0.96])
    
    # Kaydet
    output_path = os.path.join(results_dir, f'test_results_{timestamp}.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f'✅ Grafik kaydedildi: {output_path}')
    
    # --- AYRI BÜYÜK GRAFİKLER (tez için yüksek çözünürlük) ---
    # 1. Yörünge
    fig1, ax1 = plt.subplots(figsize=(10, 8))
    setup_style()
    plot_trajectory(ax1, data)
    fig1.savefig(os.path.join(results_dir, f'trajectory_{timestamp}.png'), dpi=200, bbox_inches='tight')
    
    # 2. Konum Hatası
    fig2, ax2 = plt.subplots(figsize=(12, 5))
    setup_style()
    plot_position_error(ax2, data, events)
    fig2.savefig(os.path.join(results_dir, f'position_error_{timestamp}.png'), dpi=200, bbox_inches='tight')
    
    # 3. Mod Geçişleri
    fig3, ax3 = plt.subplots(figsize=(12, 4))
    setup_style()
    plot_mode_timeline(ax3, data, events)
    fig3.savefig(os.path.join(results_dir, f'mode_timeline_{timestamp}.png'), dpi=200, bbox_inches='tight')
    
    plt.close('all')
    
    print(f'✅ Ayrı grafikler de kaydedildi:')
    print(f'   trajectory_{timestamp}.png')
    print(f'   position_error_{timestamp}.png')
    print(f'   mode_timeline_{timestamp}.png')
    print(f'\n📁 Tüm çıktılar: {results_dir}/')


if __name__ == '__main__':
    main()
