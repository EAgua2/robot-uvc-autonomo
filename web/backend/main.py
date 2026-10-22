from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import subprocess
import cv2
import base64
import io
import threading
from PIL import Image
from ultralytics import YOLO
import torch
import sys
import time
import os
import asyncio
import requests
import json
from paramiko import SSHClient, AutoAddPolicy, AuthenticationException, SSHException
import paramiko
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image as RosImage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
import math


# --- Variables Globales ---
ros_latest_frame = None
ros_latest_stamp = -1
ros_lock = threading.Lock()
map_state = {'img': ""}
ros_node = None
_last_battery_check = 0
_battery_cache = None
latest_pose = {"x": 0, "y": 0}
# Estado global para saber si alguien está viendo el stream
viewers_frames = 0
viewers_frames_lock = threading.Lock()


# --- Nodo ROS2 Unificado ---
class RosBridgeNode(Node):
    def __init__(self):
        super().__init__('fastapi_ros2_bridge')

        # Inicializa todos los suscriptores y publishers AQUÍ SOLO UNA VEZ
        self.odom_subscription = self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, 10)
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(RosImage, '/rgb/image_raw', self.image_callback, qos_profile_sensor_data)
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map2', self.map_callback, 15)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Para barra de progreso
        self.progress_data = [0, 1]
        self.progress_sub = self.create_subscription(
            Int32MultiArray,
            '/disinfection_progress',
            self.progress_callback,
            10
        )
        
        self.get_logger().info("✅ Nodo Puente ROS2-FastAPI inicializado.")
    def image_callback(self, msg):
        global ros_latest_frame, ros_latest_stamp
        #sprint("📷 Callback de imagen recibido")
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with ros_lock:
                if stamp != ros_latest_stamp:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    ros_latest_frame = cv_img
                    ros_latest_stamp = stamp
                    filename = f"/home/eduardo/frame_debug_{int(stamp)}.jpg"
                    # Guardar solo cada cierto tiempo si quieres
                    # cv2.imwrite(filename, cv_img)
                    # self.get_logger().info(f"✅ Imagen guardada para depuración: {filename}")
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir imagen: {e}")

    def map_callback(self, msg):
        global map_state
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        img = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
        img[data == 0] = 255
        img[data == 100] = 0
        img[data == -1] = 127

        # Rotar 180 grados
        img_rotated = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        img_rotated = cv2.flip(img_rotated, 0)        

        # Guardar imagen PNG localmente para depuración
        debug_path = "/home/eduardo/map_debug.png"
        try:
            #cv2.imwrite(debug_path, img_rotated)
            self.get_logger().info(f"🗺️ Mapa guardado en: {debug_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error al guardar el mapa: {e}")

        # Convertir a base64
        _, png = cv2.imencode('.png', img_rotated)
        img_base64 = base64.b64encode(png.tobytes()).decode("utf-8")
        map_state['img'] = f"data:image/png;base64,{img_base64}"

    
    def progress_callback(self, msg):
        self.progress_data = msg.data  # [completados, total]


    def odom_callback(self, msg):
        global latest_pose
        latest_pose["x"] = msg.pose.pose.position.x
        latest_pose["y"] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        #latest_pose["yaw"] = get_yaw_from_quaternion(orientation_q)

        
    



def run_ros2_node():
    global ros_node
    rclpy.init()
    ros_node = RosBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

print("🧪 Verificando GPU...")
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"✅ GPU detectada. Usando dispositivo: {device}")

model = YOLO("yolov8n.pt")
model.fuse()
model.model.to(device)

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

app.state.latest_result = {"image": "", "boxes": [], "fps": 0, "battery": None, "charging": False}
app.state.latest_result_lock = threading.Lock()

def get_battery_percent():
    global _last_battery_check, _battery_cache
    now = time.time()
    if now - _last_battery_check > 15:
        try:
            resp = requests.get("http://192.168.3.5/percent", timeout=1)
            _battery_cache = float(resp.text.strip())
            _last_battery_check = now
        except Exception as e:
            print("Error consultando porcentaje de batería:", e)
    return _battery_cache

def inference_loop():
    print("🔁 inference_loop iniciado")
    last_processed_stamp = -1
    while True:
        with viewers_frames_lock:
            if viewers_frames == 0:
                # Si nadie está viendo, duerme y no procesa nada
                time.sleep(0.1)
                continue
        with ros_lock:
            frame = ros_latest_frame
            stamp = ros_latest_stamp

        if frame is None or stamp == last_processed_stamp:
            time.sleep(0.08)
            continue

        #print("🧠 Procesando nuevo frame")
        last_processed_stamp = stamp
        start_time = time.time()
        results = model.predict(source=frame, imgsz=224, conf=0.4, verbose=False, device=device)[0]
        boxes_data = [{
            "x1": int(box.xyxy[0][0]),
            "y1": int(box.xyxy[0][1]),
            "x2": int(box.xyxy[0][2]),
            "y2": int(box.xyxy[0][3]),
            "cls": model.names[int(box.cls[0])],
            "score": round(float(box.conf[0]), 2)
        } for box in results.boxes]

        img_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        buffer = io.BytesIO()
        img_pil.save(buffer, format="JPEG", quality=35)
        img_base64 = base64.b64encode(buffer.getvalue()).decode("utf-8")
        fps = 1.0 / (time.time() - start_time)
        battery_percent = get_battery_percent()
        with app.state.latest_result_lock:
            app.state.latest_result = {
                "image": f"data:image/jpeg;base64,{img_base64}",
                "boxes": boxes_data,
                "fps": round(fps, 1),
                "battery": battery_percent,
                "charging": False
            }

def ejecutar_trayectoria_por_odometria():
    if ros_node is None:
        print("❌ Nodo ROS no está inicializado.")
        return

    def distancia(x1, y1, x2, y2):
        return ((x1 - x2)**2 + (y1 - y2)**2) ** 0.5

    def clamp(val, min_val, max_val):
        return max(min(val, max_val), min_val)

    def trayectoria():
        print("▶️ Ejecutando trayectoria mixta por odometría y giros temporizados...")
        twist = Twist()
        secuencia = [
            {"x": -0.3, "y": 0.0},
            {"x": 0, "y": 0.6},
            {"x": 0, "y": 0.6},
            {"x": 2.0, "y": 0.6},
            {"x": 2.0, "y": -0.6},
            {"x": 1.0, "y": -0.6},
            {"x": 1.0, "y": 0},
            {"giro": 1.0, "tiempo": 25.0},
            {"x": 0.2, "y": 0.0},
        ]
        k_lin = 0.6    # Ganancia proporcional
        dt = 0.05      # Intervalo entre comandos (segundos)

        VEL_MAX_X = 0.5  # metros/segundo
        VEL_MAX_Y = 0.5  # metros/segundo

        global latest_pose

        for paso in secuencia:
            # Si es un paso de posición (avance)
            if "x" in paso and "y" in paso:
                while True:
                    x_actual = latest_pose["x"]
                    y_actual = latest_pose["y"]
                    error_x = paso["x"] - x_actual
                    error_y = paso["y"] - y_actual
                    error = distancia(x_actual, y_actual, paso["x"], paso["y"])
                    if error < 0.05:   # 5 cm de tolerancia
                        break
                    vx = clamp(k_lin * error_x, -VEL_MAX_X, VEL_MAX_X)
                    vy = clamp(k_lin * error_y, -VEL_MAX_Y, VEL_MAX_Y)
                    twist.linear.x = vx
                    twist.linear.y = vy
                    twist.angular.z = 0.0
                    ros_node.cmd_vel_publisher.publish(twist)
                    print(f"x: {x_actual:.2f} y: {y_actual:.2f} vx: {vx:.2f} vy: {vy:.2f}")
                    time.sleep(dt)
                # Detenerse al llegar a cada punto
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                ros_node.cmd_vel_publisher.publish(twist)
                time.sleep(0.5)  # Pausa antes de ir al siguiente paso

            # Si es un paso de rotación por tiempo
            elif "giro" in paso and "tiempo" in paso:
                print(f"🔁 Rotando {paso['giro']} rad/s durante {paso['tiempo']} segundos...")
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = paso["giro"]
                t0 = time.time()
                while time.time() - t0 < paso["tiempo"]:
                    ros_node.cmd_vel_publisher.publish(twist)
                    time.sleep(dt)
                # Detener giro al terminar
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                ros_node.cmd_vel_publisher.publish(twist)
                time.sleep(0.2)

            else:
                print("❓ Paso no reconocido:", paso)

        print("✅ Trayectoria mixta completada.")

    threading.Thread(target=trayectoria, daemon=True).start()



@app.websocket("/ws_frames")
async def websocket_frames(websocket: WebSocket):
    global viewers_frames
    await websocket.accept()
    print("🟢 WebSocket de frames conectado.")
    with viewers_frames_lock:
        viewers_frames += 1
    try:
        while True:
            await asyncio.sleep(0.1)
            with app.state.latest_result_lock:
                result = app.state.latest_result.copy()
            await websocket.send_json(result)
    except Exception as e:
        print(f"🔴 WebSocket de frames desconectado: {e}")
    finally:
        print("Cliente de frames desconectado.")
        with viewers_frames_lock:
            viewers_frames -= 1


@app.websocket("/ws_map")
async def websocket_map(websocket: WebSocket):
    await websocket.accept()
    print("🟢 WebSocket de mapa conectado.")
    try:
        while True:
            await asyncio.sleep(0.5)
            await websocket.send_json({'image': map_state['img']})
    except Exception as e:
        print(f"🔴 WebSocket de mapa desconectado: {e}")
    finally:
        print("Cliente de mapa desconectado.")

@app.websocket("/ws_commands")
async def websocket_commands(websocket: WebSocket):
    await websocket.accept()
    print("🟢 WebSocket de comandos (teleop) conectado.")
    try:
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)
            if data.get('topic') == '/cmd_vel' and ros_node is not None:
                twist_msg = Twist()
                linear = data.get('msg', {}).get('linear', {})
                angular = data.get('msg', {}).get('angular', {})
                twist_msg.linear.x = float(linear.get('x', 0.0))
                twist_msg.linear.y = -float(linear.get('y', 0.0))
                twist_msg.angular.z = -float(angular.get('z', 0.0))
                ros_node.cmd_vel_publisher.publish(twist_msg)
                ros_node.get_logger().info(f'Publicando en /cmd_vel: L={twist_msg.linear.x}, A={twist_msg.angular.z}')
    except Exception as e:
        print(f"🔴 WebSocket de comandos desconectado: {e}")
    finally:
        print("Cliente de Teleop desconectado.")

@app.websocket("/ws_progress")
async def websocket_progress(websocket: WebSocket):
    await websocket.accept()
    print("🟢 WebSocket de progreso conectado.")
    try:
        last_sent = [-1, -1]
        while True:
            await asyncio.sleep(0.1)
            if ros_node:
                current = ros_node.progress_data
                if current != last_sent:
                    await websocket.send_json({"completed": current[0], "total": current[1]})
                    last_sent = current[:]
    except Exception as e:
        print(f"🔴 WebSocket de progreso desconectado: {e}")
    finally:
        print("Cliente de progreso desconectado.")

@app.websocket("/ws_pose")
async def websocket_pose(websocket: WebSocket):
    await websocket.accept()
    print("🟢 WebSocket de pose conectado.")
    try:
        while True:
            await asyncio.sleep(0.05)
            await websocket.send_json(latest_pose)
    except Exception as e:
        print(f"🔴 WebSocket de pose desconectado: {e}")
    finally:
        print("Cliente de pose desconectado.")



@app.on_event("startup")
def startup_event():
    print("🚀 Evento de inicio ejecutado")
    run_ros2_node()
    print("✅ Nodo ROS ejecutándose")
    threading.Thread(target=inference_loop, daemon=True).start()
    print("✅ Loop de inferencia iniciado")

@app.on_event("shutdown")
def shutdown_event():
    if ros_node:
        ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("ROS2 y FastAPI apagados correctamente.")

@app.post("/iniciar_trayectoria")
def iniciar_trayectoria():
    ejecutar_trayectoria_por_odometria()
    return {"ok": True, "msg": "Trayectoria iniciada"}



if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=False)
