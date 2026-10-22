"""
Demo del planificador de cobertura SIN ROS.

Genera un cuarto de ejemplo con obstáculos, calcula la ruta de cobertura
(búsqueda de cobertura + A* hacia la celda libre más cercana) y guarda
una imagen con el resultado.

Uso:
    pip install numpy matplotlib
    python demo/demo_cobertura.py
"""
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "ros2_ws", "src", "uvc_robot", "uvc_robot"))
from coverage_planner import CoveragePlanner, PlannerStatus  # noqa: E402

# 0 = libre, 1 = obstáculo, 2 = punto de inicio
mapa = np.zeros((20, 28), dtype=int)
mapa[0, :] = mapa[-1, :] = mapa[:, 0] = mapa[:, -1] = 1   # paredes
mapa[5:11, 8:12] = 1                                      # mesa
mapa[12:19, 18:20] = 1                                    # mueble
mapa[3:6, 20:24] = 1                                      # silla
mapa[1, 1] = 2                                            # inicio

planner = CoveragePlanner(mapa)
planner.start(initial_orientation=0)
estado = planner.compute()
encontrado, pasos, costo, trayectoria, xy = planner.result()

libres = int(np.sum(mapa != 1))
visitadas = len({(p[0], p[1]) for p in xy})
print(f"Estado final: {estado.name}")
print(f"Pasos: {pasos} | Costo: {costo:.1f} | Cobertura: {visitadas}/{libres} celdas "
      f"({100 * visitadas / libres:.1f} %)")

# ---- Figura ----
fig, ax = plt.subplots(figsize=(9, 6.5), dpi=130)
ax.imshow(mapa == 1, cmap="Greys", origin="upper")
ys = [p[1] for p in xy]
xs = [p[0] for p in xy]
ax.plot(ys, xs, color="#7c3aed", linewidth=1.8, label="Ruta de cobertura")

# Tramos de reconexión A*
for ann in planner.current_trajectory_annotations:
    if ann[2] == "A*":
        ax.plot(ann[1], ann[0], "o", color="#f59e0b", markersize=6)
ax.plot([], [], "o", color="#f59e0b", label="Reconexión con A*")
ax.plot(ys[0], xs[0], "s", color="#16a34a", markersize=9, label="Inicio")
ax.plot(ys[-1], xs[-1], "X", color="#dc2626", markersize=10, label="Fin")

ax.set_title(f"Planificador de cobertura — {100 * visitadas / libres:.0f} % del área libre cubierta")
ax.set_xticks([])
ax.set_yticks([])
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=4, frameon=False)
fig.tight_layout()

salida = os.path.join(AQUI, "..", "docs", "img", "demo_cobertura.png")
fig.savefig(salida, bbox_inches="tight")
print(f"Imagen guardada en {os.path.normpath(salida)}")
