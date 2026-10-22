import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import re


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Parámetros
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Conectado a {port} a {baud} baudios")
        except Exception as e:
            self.get_logger().error(f"Error al conectar: {str(e)}")
            raise

        # Suscripciones y publicaciones
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Variables para cálculo de velocidades
        self.last_odom_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        # Timer para leer del puerto serial
        self.timer = self.create_timer(0.001, self.read_serial)

    def cmd_vel_callback(self, msg):
        cmd_str = f"[{msg.linear.x},{msg.linear.y},{msg.angular.z}]\n"
        try:
            self.serial.write(cmd_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error al enviar: {str(e)}")

    def read_serial(self):
        if self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode('utf-8', 'ignore').strip()
                if line and "ODOM:[" in line:
                    start = line.find("ODOM:[") + 6
                    end = line.find("]", start)
                    odom_data = line[start:end].split(',')

                    odom_data = [re.sub(r'[^0-9\.\-eE]', '', s) for s in odom_data]

                    if len(odom_data) == 3:
                        try:
                            x = float(odom_data[0])
                            y = float(odom_data[1])
                            theta = float(odom_data[2])
                            stamp = self.get_clock().now().to_msg()
                            self.publish_odometry(x, y, theta, stamp)
                        except ValueError as e:
                            self.get_logger().warn(f"Error parsing odometry: {e}")
            except Exception as e:
                self.get_logger().error(f"Error al leer: {str(e)}")

    def publish_odometry(self, x, y, theta, stamp):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt == 0:
            dt = 1e-6  # evitar división por cero

        vx = (x - self.last_x) / dt
        vy = (y - self.last_y) / dt
        wz = (theta - self.last_theta) / dt

        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_odom_time = now

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        odom_msg.pose.covariance = [
            0.5, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.5, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  9999, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  9999, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  9999, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  9999
        ]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        self.odom_pub.publish(odom_msg)

        # NO se publica TF (solo se calcula internamente)
        # Si decides publicar TF más adelante, descomenta esto:
        # t = TransformStamped()
        # t.header.stamp = stamp
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'
        # t.transform.translation.x = x
        # t.transform.translation.y = y
        # t.transform.translation.z = 0.0
        # t.transform.rotation = odom_msg.pose.pose.orientation
        # self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
