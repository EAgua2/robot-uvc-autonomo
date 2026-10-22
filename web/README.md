# Interfaz web

Panel para monitorear y teleoperar el robot desde el navegador.

| Carpeta | Tecnología | Función |
|---|---|---|
| `backend/` | FastAPI + ROS 2 + YOLOv8 | Puente entre ROS 2 y el navegador por WebSockets. Detecta objetos con YOLOv8n sobre la imagen del Kinect. |
| `frontend/` | React + React Router + Tailwind CSS | Páginas Inicio, Panel principal, Control remoto y Configuración |

**WebSockets del backend:** `/ws_frames` (video, detecciones de YOLO, FPS y batería) · `/ws_map` (mapa) · `/ws_commands` (teleoperación con `/cmd_vel`) · `/ws_progress` (progreso de desinfección) · `/ws_pose` (odometría)

```bash
# Backend (con ROS 2 Jazzy cargado)
cd backend && pip install -r requirements.txt && python main.py

# Frontend
cd frontend && npm install && npm start
```

> ⚠️ Faltan las páginas `src/pages/` (`HomePage`, `DashboardPage`, `RemoteControlPage`, `SettingsPage`) y `MainLayout`. Sin ellas, el frontend no compila.
