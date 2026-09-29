"""
Taller Segundo Corte - Inciso (a): análisis del vuelo
Lee un registro vuelo_*.csv de logs/ (por defecto el más reciente), imprime los
eventos de la misión y los tiempos/errores de llegada a A, B y C, y guarda:
  - vuelo_*_trayectoria.png : trayectoria XY de los 6 drones con los puntos A, B, C
  - vuelo_*_tiempo.png      : altura, error de posición y estado en el tiempo

Uso:
    python analizar_vuelo.py
    python analizar_vuelo.py logs/vuelo_XXXX.csv
"""

import csv
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
PUNTOS = {"A": (0.0, 0.0), "B": (1.2, 1.0), "C": (1.2, -1.2)}
COLORES = {"A": "#d62728", "B": "#1f77b4", "C": "#2ca02c"}
ESTADOS = {"EN TIERRA": "#e5e7eb", "DESPEGANDO": "#fde68a", "EN VUELO": "#dbeafe", "ATERRIZANDO": "#fecaca"}
N_DRONES = 6


def main():
    if len(sys.argv) > 1:
        ruta = sys.argv[1]
    else:
        registros = sorted(glob.glob(os.path.join(AQUI, "logs", "vuelo_*.csv")))
        if not registros:
            print("No hay registros en logs/. Ejecuta primero drones_esp32.py.")
            return
        ruta = registros[-1]
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    col = lambda k: [float(r[k]) for r in filas]
    t = col("t_s")

    print(f"Registro: {os.path.basename(ruta)}  ({t[-1]:.1f} s de simulación)")
    print("Eventos:")
    for r in filas:
        if r["evento"]:
            print(f"  {float(r['t_s']):7.2f} s  {r['evento']}")

    # Tiempo de viaje entre puntos: desde que cambia el destino hasta la llegada
    print("Tramos:")
    t_salida, destino_prev = None, None
    for r in filas:
        if r["destino"] != destino_prev:
            t_salida, destino_prev = float(r["t_s"]), r["destino"]
        if r["evento"].startswith("llegada"):
            print(f"  hacia {r['destino']}: {float(r['t_s']) - t_salida:5.2f} s, "
                  f"error final del centro {float(r['error_m']) * 100:.1f} cm, "
                  f"altura {float(r['centro_z']):.2f} m")

    base = os.path.splitext(ruta)[0]

    # ---- Figura 1: trayectoria en el plano XY ----
    fig, ax = plt.subplots(figsize=(7.5, 7))
    for j in range(N_DRONES):
        xs, ys = col(f"d{j}_x"), col(f"d{j}_y")
        ax.plot(xs, ys, lw=1, alpha=0.8, label=f"dron {j}")
    ax.plot(col("centro_x"), col("centro_y"), "k--", lw=1.5, label="centro de la formación")
    for nombre, (x, y) in PUNTOS.items():
        ax.scatter([x], [y], s=350, color=COLORES[nombre], alpha=0.35, zorder=3)
        ax.annotate(nombre, (x, y), fontsize=16, fontweight="bold", ha="center", va="center",
                    color=COLORES[nombre], zorder=4)
    ax.scatter([0], [2], s=900, color="#555", alpha=0.25)
    ax.annotate("esfera", (0, 2), ha="center", va="center", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Trayectoria de la formación (vista superior)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(base + "_trayectoria.png", dpi=130)

    # ---- Figura 2: variables en el tiempo ----
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    estados = [r["estado"] for r in filas]
    for a in ax:
        inicio = 0
        for k in range(1, len(estados) + 1):
            if k == len(estados) or estados[k] != estados[inicio]:
                a.axvspan(t[inicio], t[k - 1], color=ESTADOS.get(estados[inicio], "#fff"), alpha=0.6, lw=0)
                inicio = k
        for r in filas:
            if r["evento"].startswith("llegada"):
                a.axvline(float(r["t_s"]), color=COLORES.get(r["destino"], "k"), ls=":", lw=1.2)

    ax[0].plot(t, col("ref_z"), "r", lw=1.2, label="referencia")
    ax[0].plot(t, col("centro_z"), "b", lw=1.2, label="centro de la formación")
    ax[0].set_ylabel("altura (m)")
    ax[0].legend(loc="upper right", fontsize=8)

    ax[1].plot(t, col("ref_x"), "r--", lw=1, label="ref x")
    ax[1].plot(t, col("centro_x"), "r", lw=1.2, label="centro x")
    ax[1].plot(t, col("ref_y"), "b--", lw=1, label="ref y")
    ax[1].plot(t, col("centro_y"), "b", lw=1.2, label="centro y")
    ax[1].set_ylabel("posición (m)")
    ax[1].legend(loc="upper right", fontsize=8, ncol=2)

    ax[2].plot(t, [e * 100 for e in col("error_m")], "k", lw=1.2)
    ax[2].set_ylabel("distancia al destino (cm)")
    ax[2].set_xlabel("tiempo (s)")
    handles = [plt.Rectangle((0, 0), 1, 1, color=c, alpha=0.6) for c in ESTADOS.values()]
    ax[2].legend(handles, ESTADOS.keys(), loc="upper right", fontsize=8, ncol=4)
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("Misión A → B → C (líneas punteadas = llegada a cada punto)")
    fig.tight_layout()
    fig.savefig(base + "_tiempo.png", dpi=130)
    print(f"Figuras guardadas:\n  {base}_trayectoria.png\n  {base}_tiempo.png")


if __name__ == "__main__":
    main()
