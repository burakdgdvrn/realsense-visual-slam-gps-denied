import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Tüm düğümlerde ortak kullanılacak kamera ve zaman parametreleri
    base_params = {
        'use_sim_time': True,
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'approx_sync': True,
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_depth': 2,
        'qos_odom': 2
    }
    
    # ─────────────────────────────────────────────────────
    # GÖRSEL ODOMETRİ (VO) PARAMETRELERİ
    # ─────────────────────────────────────────────────────
    odom_params = base_params.copy()
    odom_params.update({
        # TF Yayını: odom → base_link
        # Bu artık TEK kaynak — hybrid_localizer TF yayınlamıyor.
        'publish_tf': True,
        'odom_frame_id': 'odom',
        
        # Feature Dedektör: ORB (hızlı ve hafif — SURF'ten 10x daha verimli)
        'Kp/DetectorStrategy': '6',
        'Kp/MaxFeatures': '300',         # Feature sayısını sınırla (bellek tasarrufu)
        
        # Odometri Stratejisi: Frame-to-Map (drift çok daha az)
        'Odom/Strategy': '0',            # 0=F2M (Frame-to-Map), 1=F2F (Frame-to-Frame)
        'Vis/CorType': '0',              # 0=Feature-based matching (Gazebo sentetik görüntülerinde daha kararlı)
        'Odom/ResetCountdown': '5',      # 5 kötü frame'e tolerans (1'de anında reset → TF tutarsızlığı)
        'Odom/GuessMotion': 'true',      # Dönüşlerde kameranın hareketini tahmin et
    })

    # ─────────────────────────────────────────────────────
    # RTAB-Map SLAM PARAMETRELERİ
    # ─────────────────────────────────────────────────────
    slam_params = base_params.copy()
    slam_params.update({
        'subscribe_odom_info': False,     # odom_info topic remap sorunu önlenir
        
        # Feature Dedektör (SLAM tarafı da aynı)
        'Kp/DetectorStrategy': '6',       # ORB
        'Kp/MaxFeatures': '300',
        
        # Harita ve Grid
        'Grid/3D': 'true',
        'Grid/MaxObstacleHeight': '2.5',
        
        # Bellek Yönetimi — Çökmeyi Önleyen Parametreler
        'Rtabmap/DetectionRate': '2.0',   # Saniyede max 2 frame işle (30 yerine!)
        'Mem/STMSize': '15',              # Kısa-dönem hafıza boyutu (varsayılan 30 → 15)
        
        # Loop Closure Kalite Kontrolü
        'RGBD/OptimizeMaxError': '3.0',   # Max 3m hata toleransı (yanlış LC'leri filtrele)
        'Rtabmap/LoopThr': '0.11',        # Loop closure benzerlik eşiği
        'RGBD/ProximityBySpace': 'false', # Spatial proximity LC kapalı (tekrarlı dokular yanlış tetikliyor)
    })

    # Kamera Topic'leri
    remappings = [
        ('rgb/image', '/master/camera/image_raw'),
        ('rgb/camera_info', '/master/camera/camera_info'),
        ('depth/image', '/master/camera/depth/image_raw')
    ]

    return LaunchDescription([
        # 1. Statik TF: base_link → realsense_link (Kamera Konumu)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=['0.22', '0.0', '0.0', '-0.5', '0.5', '-0.5', '0.5', 'base_link', 'realsense_link']
        ),
        
        # 2. GÖRSEL ODOMETRİ (VO) MOTORU
        # TF zinciri: odom → base_link (TEK KAYNAK — hybrid_localizer artık TF yayınlamıyor)
        # GPS kesildiğinde hybrid_localizer bu /rtabmap/vo topic'ini okuyarak füzyon yapacak.
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=[odom_params],
            remappings=remappings + [('odom', '/rtabmap/vo')]
        ),
        
        # 3. HARİTACI (RTAB-Map SLAM)
        # TF zinciri: map → odom
        # Kendi bağımsız VO'sunu (/rtabmap/vo) kullanır — füzyon çıktısına bağımlı DEĞİL.
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[slam_params],
            remappings=remappings + [('odom', '/rtabmap/vo')],
            arguments=['-d']   # Her başlatmada yeni veritabanı
        ),
        
        # 4. GÖRSELLEŞTİRİCİ
        # Not: Performans sorunu yaşanırsa bu node yorum satırına alınabilir.
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[base_params],
            remappings=remappings + [('odom', '/rtabmap/vo')]
        )
    ])