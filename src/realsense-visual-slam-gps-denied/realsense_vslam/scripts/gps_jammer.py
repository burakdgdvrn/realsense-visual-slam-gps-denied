#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class GpsJammer(Node):
    def __init__(self):
        super().__init__('gps_jammer')
        self.publisher_ = self.create_publisher(Bool, '/system/gps_status', 10)
        
        print("========================================")
        print("  🛰️  GPS SİBER SALDIRI SİMÜLATÖRÜ  🛰️")
        print("========================================")
        print("[ENTER] tuşuna basarak GPS'i KES / GERİ GETİR")
        print("Çıkmak için CTRL+C")
        print("========================================")
        
        self.gps_active = True
        
        # İlk durumu yayınla
        self.publish_status()
        
        # Kullanıcı girdisini bekle
        self.create_timer(0.5, self.check_input)

    def publish_status(self):
        msg = Bool()
        msg.data = self.gps_active
        self.publisher_.publish(msg)

    def check_input(self):
        # Python'da non-blocking input almak zordur, bu yüzden basit bir input kullanacağız
        # Not: input() fonksiyonu thread'i bloke edebilir ancak bu sadece bir tetikleyici script.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GpsJammer()
    
    # Kendi input döngümüzü kuralım
    import threading
    
    def input_thread():
        while rclpy.ok():
            try:
                input() # ENTER bekler
                node.gps_active = not node.gps_active
                if node.gps_active:
                    node.get_logger().info("🟢 GPS SİNYALİ GERİ GELDİ!")
                else:
                    node.get_logger().error("🔴 SİBER SALDIRI: GPS SİNYALİ KESİLDİ!")
                node.publish_status()
            except (EOFError, KeyboardInterrupt):
                break
                
    thread = threading.Thread(target=input_thread, daemon=True)
    thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
