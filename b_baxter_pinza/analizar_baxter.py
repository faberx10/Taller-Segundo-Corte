"""
Taller Segundo Corte - Inciso (b): análisis de una sesión con el Baxter
Lee un registro baxter_*.csv de logs/ (por defecto el más reciente), imprime los
eventos y un resumen (agarres, bloques en la bandeja, error de seguimiento,
velocidades) y guarda tres figuras junto al registro:
  - baxter_*_seguimiento.png : objetivo de la consola, trayectoria suave y posición
                               real de cada pinza (x, y, z) en el tiempo
  - baxter_*_mesa.png        : vista superior de la mesa con las trayectorias de
                               las pinzas y de los bloques
  - baxter_*_fluidez.png     : rapidez de cada pinza y error de seguimiento

Uso:
    python analizar_baxter.py
    python analizar_baxter.py logs/baxter_XXXX.csv
    python analizar_baxter.py logs/baxter_XXXX.csv --desde 60 --hasta 180   (solo ese tramo)
"""

import argparse
import csv
import glob
import io
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

AQUI = os.path.dirname(os.path.abspath(__file__))
TOP = -0.18
BANDEJA = ((0.66, 0.0), 0.08)
MESA = ((0.62, 0.0), (0.25, 0.62))

# Paleta (validada con el validador de la guía de visualización: CVD y contraste OK)
SUPERFICIE, TEXTO, TEXTO_2, REJILLA = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
AZUL, NARANJA, ROJO = "#2a78d6", "#eb6834", "#e34948"
COLOR_BRAZO = {"der": ROJO, "izq": AZUL}
COLOR_BLOQUE = {"rojo": ROJO, "azul": AZUL}
NOMBRE = {"der": "derecho", "izq": "izquierdo"}

plt.rcParams.update({
    "figure.facecolor": SUPERFICIE, "axes.facecolor": SUPERFICIE, "savefig.facecolor": SUPERFICIE,
    "axes.edgecolor": REJILLA, "axes.labelcolor": TEXTO_2, "xtick.color": TEXTO_2, "ytick.color": TEXTO_2,
    "text.color": TEXTO, "axes.titlecolor": TEXTO, "axes.grid": True, "grid.color": REJILLA,
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 2.0, "font.size": 9, "legend.frameon": False,
})



def abrir_csv(ruta):
    """Abre el registro como texto. Los registros nuevos son UTF-8; los que se
    grabaron en Windows antes de fijar la codificación están en cp1252."""
    with open(ruta, "rb") as f:
        datos = f.read()
    try:
        texto = datos.decode("utf-8")
    except UnicodeDecodeError:
        texto = datos.decode("cp1252")
    return io.StringIO(texto, newline="")

def cargar(ruta):
    with abrir_csv(ruta) as f:
        filas = list(csv.DictReader(f))
    datos = {k: [r[k] for r in filas] for k in filas[0]}
    num = {}
    for k, v in datos.items():
        try:
            num[k] = np.array([float(x) for x in v])
        except ValueError:
            num[k] = np.array(v)
    return num


def intervalos(mascara, t):
    """Tramos [t_ini, t_fin] donde la máscara es verdadera."""
    tramos, ini = [], None
    for k, m in enumerate(mascara):
        if m and ini is None:
            ini = t[k]
        if not m and ini is not None:
            tramos.append((ini, t[k]))
            ini = None
    if ini is not None:
        tramos.append((ini, t[-1]))
    return tramos


def main():
    ap = argparse.ArgumentParser(description="Análisis de una sesión (baxter_*.csv)")
    ap.add_argument("registro", nargs="?", help="archivo CSV (por defecto, el más reciente de logs/)")
    ap.add_argument("--desde", type=float, default=None, help="analizar desde este instante (s)")
    ap.add_argument("--hasta", type=float, default=None, help="analizar hasta este instante (s)")
    args = ap.parse_args()
    if args.registro:
        ruta = args.registro
    else:
        registros = sorted(glob.glob(os.path.join(AQUI, "logs", "baxter_*.csv")))
        if not registros:
            print("No hay registros en logs/. Ejecuta primero baxter_pinza_esp32.py.")
            return
        ruta = registros[-1]
    d = cargar(ruta)
    ventana = np.ones(len(d["t_s"]), dtype=bool)
    if args.desde is not None:
        ventana &= d["t_s"] >= args.desde
    if args.hasta is not None:
        ventana &= d["t_s"] <= args.hasta
    d = {k: v[ventana] for k, v in d.items()}
    t = d["t_s"]
    dt = float(np.median(np.diff(t)))
    base = os.path.splitext(ruta)[0]

    # ---------------- Resumen en texto ----------------
    print(f"Registro: {os.path.basename(ruta)}  ({t[-1]:.1f} s)")
    print("Eventos:")
    for ti, ev in zip(t, d["evento"]):
        if ev:
            print(f"  {ti:7.2f} s  {ev}")
    agarres = sum(ev.count("sujeta el bloque") for ev in d["evento"])
    print(f"\nAgarres: {agarres}")
    for nombre in COLOR_BLOQUE:
        x, y, z = (d[f"bloque_{nombre}_{c}"][-1] for c in "xyz")
        dentro = abs(x - BANDEJA[0][0]) < BANDEJA[1] and abs(y - BANDEJA[0][1]) < BANDEJA[1] and z < TOP + 0.05
        print(f"Bloque {nombre}: final ({x:.3f}, {y:+.3f}, {z - TOP:.3f} sobre la mesa) -> "
              f"{'EN LA BANDEJA' if dentro else 'fuera de la bandeja'}")

    rapidez, error = {}, {}
    for lado in COLOR_BRAZO:
        real = np.stack([d[f"{lado}_real_{c}"] for c in "xyz"], axis=1)
        suave = np.stack([d[f"{lado}_suave_{c}"] for c in "xyz"], axis=1)
        v = np.linalg.norm(np.gradient(real, dt, axis=0), axis=1)
        v = np.convolve(v, np.ones(7) / 7, mode="same")          # media móvil de ~60 ms
        rapidez[lado], error[lado] = v, np.linalg.norm(real - suave, axis=1) * 100
        mov = v > 0.02
        e = error[lado][mov] if mov.any() else error[lado]
        acel = np.gradient(np.convolve(v, np.ones(5) / 5, mode="same"), dt)
        print(f"Pinza {NOMBRE[lado]}: rapidez máx {v.max():.2f} m/s | error de seguimiento en movimiento: "
              f"medio {e.mean():.2f} cm, p95 {np.percentile(e, 95):.2f} cm, máx {e.max():.2f} cm | "
              f"aceleración máx {np.abs(acel).max():.2f} m/s²")

    # ---------------- Figura 1: seguimiento ----------------
    fig, ax = plt.subplots(3, 2, figsize=(12, 7.5), sharex=True)
    for col, lado in enumerate(COLOR_BRAZO):
        cerrada = d[f"{lado}_pinza_abierta"] < 0.5
        for fila, c in enumerate("xyz"):
            a = ax[fila, col]
            for ini, fin in intervalos(cerrada, t):
                a.axvspan(ini, fin, color="#f1efe8", lw=0)
            desplaz = TOP if c == "z" else 0.0
            a.plot(t, d[f"{lado}_obj_{c}"] - desplaz, color=TEXTO_2, lw=1.0, ls="--", label="objetivo (consola)")
            a.plot(t, d[f"{lado}_suave_{c}"] - desplaz, color=AZUL, lw=2.0, label="trayectoria suave")
            a.plot(t, d[f"{lado}_real_{c}"] - desplaz, color=NARANJA, lw=1.2, label="pinza real")
            a.set_ylabel(f"{c} (m)" + (" sobre la mesa" if c == "z" else ""))
            if fila == 0:
                a.set_title(f"Brazo {NOMBRE[lado]}  (fondo gris = pinza cerrada)", loc="left", fontsize=10)
    ax[2, 0].set_xlabel("tiempo (s)")
    ax[2, 1].set_xlabel("tiempo (s)")
    fig.suptitle("Seguimiento de la pinza: consola → trayectoria suave → robot", x=0.01, ha="left", fontsize=12)
    fig.legend(*ax[0, 0].get_legend_handles_labels(), loc="upper right", ncol=3, fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(base + "_seguimiento.png", dpi=130)

    # ---------------- Figura 2: vista superior ----------------
    fig, a = plt.subplots(figsize=(9, 5.6))
    (mx, my), (hx, hy) = MESA          # en la figura: eje horizontal = y, vertical = x
    a.add_patch(Rectangle((my - hy, mx - hx), 2 * hy, 2 * hx, color="#efe6d8", zorder=0, lw=0))
    (bx, by), bm = BANDEJA
    a.add_patch(Rectangle((by - bm, bx - bm), 2 * bm, 2 * bm, fill=False, ec="#2e8b3e", lw=2, zorder=1))
    a.annotate("bandeja", (by, bx + bm + 0.012), ha="center", color=TEXTO_2)
    a.annotate("mesa", (my + hy - 0.02, mx + hx - 0.03), ha="left", color=TEXTO_2)
    for lado, color in COLOR_BRAZO.items():
        a.plot(d[f"{lado}_real_y"], d[f"{lado}_real_x"], color=color, lw=1.0, alpha=0.55,
               label=f"pinza {NOMBRE[lado]}")
    for nombre, color in COLOR_BLOQUE.items():
        x, y = d[f"bloque_{nombre}_x"].copy(), d[f"bloque_{nombre}_y"].copy()
        salto = np.r_[False, np.hypot(np.diff(x), np.diff(y)) > 0.05]   # reinicio de los bloques
        x[salto], y[salto] = np.nan, np.nan
        a.plot(y, x, color=color, lw=3.0, label=f"bloque {nombre}")
        a.scatter([y[0]], [x[0]], s=90, marker="s", facecolor=SUPERFICIE, edgecolor=color, lw=2, zorder=3)
        a.scatter([y[-1]], [x[-1]], s=90, marker="s", color=color, edgecolor=SUPERFICIE, lw=2, zorder=3)
    a.scatter([], [], s=60, marker="s", facecolor=SUPERFICIE, edgecolor=TEXTO_2, label="posición inicial")
    a.scatter([], [], s=60, marker="s", color=TEXTO_2, label="posición final")
    a.set_xlim(my + hy, my - hy)          # como se ve desde el frente del robot
    a.set_ylim(0.34, mx + hx + 0.03)
    a.set_aspect("equal")
    a.set_xlabel("y (m)  ← izquierda del robot | derecha del robot →")
    a.set_ylabel("x (m)  (distancia al robot)")
    a.set_title("Vista superior de la mesa", loc="left", fontsize=11)
    a.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8)
    fig.tight_layout()
    fig.savefig(base + "_mesa.png", dpi=130)

    # ---------------- Figura 3: fluidez ----------------
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.8), sharex=True)
    for lado, color in COLOR_BRAZO.items():
        ax[0].plot(t, rapidez[lado], color=color, lw=1.5, label=f"pinza {NOMBRE[lado]}")
        ax[1].plot(t, error[lado], color=color, lw=1.2, label=f"pinza {NOMBRE[lado]}")
    ax[0].axhline(0.25, color=TEXTO_2, lw=1, ls="--")
    ax[0].annotate("límite en modo manual: 0.25 m/s", (t[0], 0.25), ha="left", va="bottom", color=TEXTO_2,
                   fontsize=8)
    ax[0].set_ylabel("rapidez (m/s)\nmedia móvil 60 ms")
    ax[0].set_ylim(0, 0.32)
    ax[1].set_ylabel("error real vs. suave (cm)")
    ax[1].set_xlabel("tiempo (s)")
    fig.legend(*ax[0].get_legend_handles_labels(), loc="upper right", ncol=2, fontsize=8)
    ax[0].set_title("Rapidez de las pinzas y error de seguimiento", loc="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(base + "_fluidez.png", dpi=130)
    print(f"\nFiguras guardadas:\n  {base}_seguimiento.png\n  {base}_mesa.png\n  {base}_fluidez.png")


if __name__ == "__main__":
    main()
