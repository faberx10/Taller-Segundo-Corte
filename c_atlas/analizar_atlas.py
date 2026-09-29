"""
Taller Segundo Corte - Inciso (c): análisis de una sesión con el Atlas
Lee un registro atlas_*.csv de logs/ (por defecto el más reciente), imprime los
eventos y un resumen (poses alcanzadas y su error, equilibrio, caídas) y guarda
dos figuras junto al registro:
  - atlas_*_articulaciones.png : objetivo de la consola, trayectoria suave y ángulo
                                 real de los 18 grados de libertad
  - atlas_*_equilibrio.png     : centro de masa frente a los pies, margen de
                                 estabilidad y correcciones del control de tobillo

Uso:
    python analizar_atlas.py
    python analizar_atlas.py logs/atlas_XXXX.csv
"""

import csv
import glob
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
PANELES = [("Brazo derecho", [f"der_{j}" for j in ("shz", "shx", "ely", "elx", "wry", "wrx")]),
           ("Brazo izquierdo", [f"izq_{j}" for j in ("shz", "shx", "ely", "elx", "wry", "wrx")]),
           ("Torso, cabeza y piernas", ["torso_giro", "torso_adelante", "torso_lateral", "cabeza",
                                       "sentadilla", "balanceo"])]
ETIQUETA = {"torso_giro": "torso giro", "torso_adelante": "torso adelante", "torso_lateral": "torso lateral",
            "cabeza": "cabeza", "sentadilla": "sentadilla", "balanceo": "balanceo"}

# Paleta validada (misma de los incisos a y b)
SUPERFICIE, TEXTO, TEXTO_2, REJILLA = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
AZUL, NARANJA, ROJO = "#2a78d6", "#eb6834", "#e34948"
FONDO_MODO = {"REPRODUCIENDO": "#e8f0fb", "DEMO": "#fdf0e6", "CAIDO": "#fbe3e3"}
NOMBRE_MODO = {"REPRODUCIENDO": "reproduciendo poses grabadas", "DEMO": "demostración", "CAIDO": "caído (reponiendo)"}

plt.rcParams.update({
    "figure.facecolor": SUPERFICIE, "axes.facecolor": SUPERFICIE, "savefig.facecolor": SUPERFICIE,
    "axes.edgecolor": REJILLA, "axes.labelcolor": TEXTO_2, "xtick.color": TEXTO_2, "ytick.color": TEXTO_2,
    "text.color": TEXTO, "axes.titlecolor": TEXTO, "axes.grid": True, "grid.color": REJILLA,
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 2.0, "font.size": 9, "legend.frameon": False,
})


def cargar(ruta):
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    d = {}
    for k in filas[0]:
        v = [r[k] for r in filas]
        try:
            d[k] = np.array([float(x) for x in v])
        except ValueError:
            d[k] = np.array(v)
    return d


def sombrear(a, modos, t):
    ini = 0
    for k in range(1, len(modos) + 1):
        if k == len(modos) or modos[k] != modos[ini]:
            if modos[ini] in FONDO_MODO:
                a.axvspan(t[ini], t[k - 1], color=FONDO_MODO[modos[ini]], lw=0)
            ini = k


def main():
    if len(sys.argv) > 1:
        ruta = sys.argv[1]
    else:
        registros = sorted(glob.glob(os.path.join(AQUI, "logs", "atlas_*.csv")))
        if not registros:
            print("No hay registros en logs/. Ejecuta primero atlas_esp32.py.")
            return
        ruta = registros[-1]
    d = cargar(ruta)
    t = d["t_s"]
    base = os.path.splitext(ruta)[0]
    modos = [m.split(" ")[0] for m in d["modo"]]

    # ---------------- Resumen ----------------
    print(f"Registro: {os.path.basename(ruta)}  ({t[-1]:.1f} s)")
    print("Eventos:")
    errores = []
    for ti, ev in zip(t, d["evento"]):
        if ev:
            print(f"  {ti:7.2f} s  {ev}")
            m = re.search(r"alcanzada \(error máx ([0-9.]+) grados\)", ev)
            if m:
                errores.append(float(m.group(1)))
    print(f"\nPoses grabadas: {sum('grabada' in ev for ev in d['evento'])}")
    if errores:
        print(f"Poses alcanzadas (reproducción y demostraciones): {len(errores)} | error máximo por pose: "
              f"medio {np.mean(errores):.2f} grados, peor {max(errores):.2f} grados")
    de_pie = np.array([m != "CAIDO" for m in modos])
    margen = d["margen_m"] * 100
    caidas = sum("se cayó" in ev for ev in d["evento"])
    print(f"Caídas: {caidas}")
    # se descartan los 3 s previos a cada caída (ahí el margen cae a propósito)
    antes_caida = np.zeros(len(t), dtype=bool)
    for ti, ev in zip(t, d["evento"]):
        if "se cayó" in ev:
            antes_caida |= (t > ti - 3) & (t <= ti)
    estable = de_pie & ~antes_caida
    print(f"Equilibrio de pie (sin los 3 s previos a cada caída): margen mínimo {margen[estable].min():.1f} cm, "
          f"mediano {np.median(margen[estable]):.1f} cm; {100 * np.mean(margen[estable] > 3):.1f} % del tiempo "
          f"con más de 3 cm de margen")
    print(f"Corrección de tobillo: máx {np.degrees(np.abs(d['corr_adelante']).max()):.1f} grados adelante-atrás, "
          f"{np.degrees(np.abs(d['corr_lateral']).max()):.1f} grados lateral")

    # ---------------- Figura 1: grados de libertad ----------------
    fig, ax = plt.subplots(6, 3, figsize=(14, 11), sharex=True)
    for col, (titulo, nombres) in enumerate(PANELES):
        for fila, n in enumerate(nombres):
            a = ax[fila, col]
            sombrear(a, modos, t)
            escala = 100 if n == "sentadilla" else np.degrees(1)
            a.plot(t, d[f"{n}_obj"] * escala, color=TEXTO_2, lw=1.0, ls="--", label="objetivo (consola)")
            a.plot(t, d[f"{n}_suave"] * escala, color=AZUL, lw=2.0, label="trayectoria suave")
            a.plot(t, d[f"{n}_real"] * escala, color=NARANJA, lw=1.2, label="real")
            nombre = ETIQUETA[n] if n in ETIQUETA else n.split("_")[1]
            a.set_ylabel(nombre + (" (x100)" if n == "sentadilla" else " (°)"))
            if fila == 0:
                a.set_title(titulo, loc="left", fontsize=10)
    for a in ax[-1]:
        a.set_xlabel("tiempo (s)")
    handles, labels = ax[0, 0].get_legend_handles_labels()
    handles += [plt.Rectangle((0, 0), 1, 1, color=c) for c in FONDO_MODO.values()]
    labels += list(NOMBRE_MODO.values())
    fig.legend(handles, labels, loc="upper right", ncol=6, fontsize=8)
    fig.suptitle("Atlas: consola → trayectoria suave → robot", x=0.01, ha="left", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(base + "_articulaciones.png", dpi=115)

    # ---------------- Figura 2: equilibrio ----------------
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for a in ax:
        sombrear(a, modos, t)
        for ti, ev in zip(t, d["evento"]):
            if "se cayó" in ev:
                a.axvline(ti, color=ROJO, lw=1.2, ls=":")
    ex = np.where(de_pie, (d["cm_x"] - d["apoyo_cx"]) * 100, np.nan)
    ey = np.where(de_pie, (d["cm_y"] - d["apoyo_cy"]) * 100, np.nan)
    ax[0].plot(t, ex, color=AZUL, lw=1.5, label="adelante-atrás")
    ax[0].plot(t, ey, color=NARANJA, lw=1.5, label="lateral")
    ax[0].set_ylabel("CM respecto al\ncentro de los pies (cm)")
    ax[0].legend(loc="upper right", fontsize=8)
    ax[1].plot(t, np.where(de_pie, margen, np.nan), color=TEXTO, lw=1.5)
    ax[1].axhline(3, color=TEXTO_2, lw=1, ls="--")
    ax[1].axhline(0, color=ROJO, lw=1)
    ax[1].annotate("3 cm: límite de 'estable'", (t[0], 3), va="bottom", fontsize=8, color=TEXTO_2)
    ax[1].annotate("0: el CM sale de los pies", (t[0], 0), va="top", fontsize=8, color=ROJO)
    ax[1].set_ylabel("margen de estabilidad (cm)")
    ax[2].plot(t, np.degrees(d["corr_adelante"]), color=AZUL, lw=1.2, label="tobillo adelante-atrás")
    ax[2].plot(t, np.degrees(d["corr_lateral"]), color=NARANJA, lw=1.2, label="tobillo lateral")
    ax[2].set_ylabel("corrección del\ncontrol de equilibrio (°)")
    ax[2].set_xlabel("tiempo (s)")
    ax[2].legend(loc="upper right", fontsize=8)
    ax[0].set_title("Equilibrio del Atlas (línea punteada roja = caída)", loc="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(base + "_equilibrio.png", dpi=130)
    print(f"\nFiguras guardadas:\n  {base}_articulaciones.png\n  {base}_equilibrio.png")


if __name__ == "__main__":
    main()
