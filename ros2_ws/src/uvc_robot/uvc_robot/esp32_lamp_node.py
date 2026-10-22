import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import requests

# IP local de la ESP32 de monitoreo (lámparas UV-C y batería). Ajustar a tu red.
ESP32_IP = "192.168.1.83"


class ESP32ControlNode(Node):
    def __init__(self):
        super().__init__('esp32_control_node')
        self.pub = self.create_publisher(Float32, 'bateria_percent', 10)
        self.sub = self.create_subscription(String, 'encender_lampara', self.callback_lampara, 10)
        self.timer = self.create_timer(5.0, self.leer_bateria)  # cada 5 segundos

    def leer_bateria(self):
        try:
            r = requests.get(f"http://{ESP32_IP}/percent", timeout=2)
            if r.status_code == 200:
                percent = float(r.text.strip())
                msg = Float32()
                msg.data = percent
                self.pub.publish(msg)
                self.get_logger().info(f"Porcentaje batería: {percent:.1f} %")
        except Exception as e:
            self.get_logger().error(f"Error leyendo batería: {e}")

    def callback_lampara(self, msg):
        cmd = msg.data.lower()
        try:
            if "on" in cmd or "encender" in cmd:
                requests.get(f"http://{ESP32_IP}/toggle")
                self.get_logger().info("Lámpara encendida")
            elif "off" in cmd or "apagar" in cmd:
                requests.get(f"http://{ESP32_IP}/off")
                self.get_logger().info("Lámpara apagada")
            else:
                self.get_logger().info("Comando desconocido")
        except Exception as e:
            self.get_logger().error(f"Error enviando comando: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ESP32ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
