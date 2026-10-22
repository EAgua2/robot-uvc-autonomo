import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import math
from .coverage_planner import CoveragePlanner


class CoverageNode(Node):
    def __init__(self):
        super().__init__('coverage_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map2',
            self.map_callback,
            10)
        self.posearray_pub = self.create_publisher(PoseArray, '/goal_poses', 10)
        self.path_pub = self.create_publisher(Path, '/goal_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/start_marker', 10)
        self.sent = False

    def map_callback(self, msg):
        if self.sent:
            return

        resolution = msg.info.resolution
        origin = msg.info.origin
        width = msg.info.width
        height = msg.info.height

        self.get_logger().info(f"📏 Resolución del mapa: {resolution:.3f} m/celda")

        mapa_original = np.array(msg.data, dtype=np.int8).reshape((height, width))
        obstaculos = np.logical_or(mapa_original == 100, mapa_original == -1).astype(np.uint8)

        # Inflar obstáculos
        safe_distance_m = 0.25
        inflation_cells = int(safe_distance_m / resolution)
        self.get_logger().info(f"🧩 Inflando obstáculos: {inflation_cells} celdas alrededor")

        data = np.copy(mapa_original)
        for i in range(height):
            for j in range(width):
                if obstaculos[i, j] == 1:
                    x0 = max(0, i - inflation_cells)
                    x1 = min(height, i + inflation_cells + 1)
                    y0 = max(0, j - inflation_cells)
                    y1 = min(width, j + inflation_cells + 1)
                    data[x0:x1, y0:y1] = 100

        # Convertir a baja resolución (0.05 m/celda) para planificación
        target_res = 0.05
        factor = int(target_res / resolution)

        if factor > 1:
            low_res_h = height // factor
            low_res_w = width // factor
            planner_map = np.zeros((low_res_h, low_res_w), dtype=np.uint8)

            for i in range(low_res_h):
                for j in range(low_res_w):
                    block = data[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
                    if np.any(block == 100) or np.any(block == -1):
                        planner_map[i, j] = 1
                    else:
                        planner_map[i, j] = 0
        else:
            planner_map = np.where(data == 0, 0, 1)

        # Definir punto inicial (convertido a baja resolución)
        x_init = 0.0
        y_init = 0.0
        fila = int((y_init - origin.position.y) / target_res)
        columna = int((x_init - origin.position.x) / target_res)

        if 0 <= fila < planner_map.shape[0] and 0 <= columna < planner_map.shape[1] and planner_map[fila, columna] == 0:
            planner_map[fila, columna] = 2
        else:
            self.get_logger().warn("📍 Punto inicial no válido, buscando punto libre más cercano...")
            libres = np.argwhere(planner_map == 0)
            if libres.size == 0:
                self.get_logger().error("❌ No hay celdas libres disponibles.")
                return
            distancias = np.linalg.norm(libres - np.array([fila, columna]), axis=1)
            fila, columna = libres[np.argmin(distancias)]
            planner_map[fila, columna] = 2

        # Planificación
        planner = CoveragePlanner(planner_map)
        planner.start(initial_orientation=0)
        planner.compute()
        found, _, _, trayectoria, xy_traj = planner.result()

        if not trayectoria or len(trayectoria) < 2:
            self.get_logger().error("❌ Ruta de cobertura vacía o muy corta.")
            return

        # Convertir trayectoria a poses reales
        ESPACIADO_X = 0.5
        ESPACIADO_Y = 0.5
        last_px = last_py = None
        poses = []

        for i in range(len(trayectoria)):
            _, x, y, _, _, _, _ = trayectoria[i]
            px = y * target_res + origin.position.x
            py = x * target_res + origin.position.y

            if (last_px is None or
                abs(px - last_px) >= ESPACIADO_X or
                    abs(py - last_py) >= ESPACIADO_Y):

                pose = Pose()
                pose.position.x = px
                pose.position.y = py

                # Orientación hacia el siguiente punto
                if i < len(trayectoria) - 1:
                    _, nx, ny, _, _, _, _ = trayectoria[i + 1]
                    next_px = ny * target_res + origin.position.x
                    next_py = nx * target_res + origin.position.y
                    dx = next_px - px
                    dy = next_py - py
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0

                pose.orientation.z = math.sin(yaw / 2.0)
                pose.orientation.w = math.cos(yaw / 2.0)

                poses.append(pose)
                last_px = px
                last_py = py

        # Publicar PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = poses
        self.posearray_pub.publish(pose_array)
        self.get_logger().info(f"✅ Enviados {len(poses)} goal_poses seguros.")

        # Publicar Path para visualización
        path = Path()
        path.header = pose_array.header
        for p in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose = p
            path.poses.append(ps)
        self.path_pub.publish(path)

        # Publicar marcador de inicio
        if poses:
            start_pose = poses[0]
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'start_marker'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = start_pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            self.marker_pub.publish(marker)
            self.get_logger().info('🟢 Marcador de inicio publicado.')

        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
