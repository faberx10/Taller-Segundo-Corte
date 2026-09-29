"""
Taller Segundo Corte - Inciso (c): consola ESP32 para la movilidad del robot Atlas
Micros y Laboratorio - UMNG

El humanoide Atlas de Boston Dynamics (atlas_v4_with_multisense.urdf) de pie sobre
una caja en el laboratorio "botlab", como en atlas.py de erwincoumans/pybullet_robots.
El robot NO está anclado: se sostiene sobre sus pies con la física de PyBullet,
así que cada movimiento cambia su equilibrio. Un control de equilibrio en los
tobillos mantiene su centro de masa sobre los pies y un indicador muestra qué tan
cerca está de caerse; si se cae, se repone solo.

La consola mueve 18 grados de libertad en 4 grupos de 3 (uno por potenciómetro):
    Grupo 1 TORSO Y CABEZA    : giro del torso, inclinación adelante, cabeza
    Grupo 2 HOMBRO            : shz, shx, ely del brazo elegido
    Grupo 3 CODO Y MUÑECA     : elx, wry, wrx del brazo elegido
    Grupo 4 PIERNAS           : sentadilla, balanceo lateral, inclinación lateral del torso
          (las piernas se mueven coordinadas: cadera, rodilla y tobillo a la vez)

    Pulsador 1 -> siguiente grupo
    Pulsador 2 -> brazo: DERECHO -> IZQUIERDO -> AMBOS (espejo)
    Pulsador 3 -> grabar la pose actual (mantener 1.5 s o tecla C: borrar todas)
    Pulsador 4 -> reproducir / detener las poses grabadas (en bucle)
    Pulsador 5 -> siguiente demostración: saludo, sentadilla, giro de torso, brazos abajo, en cruz
    Tecla R    -> reponer el Atlas de pie sobre la caja

Uso:
    python atlas_esp32.py                 # consola ESP32 en COM5
    python atlas_esp32.py --port COM7
    python atlas_esp32.py --sin-serial    # consola virtual (sliders)
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import pybullet as p

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)
from comun.consola import ConsolaESP32, ConsolaVirtual, Hud, adc_a_rango  # noqa: E402

MODELOS = os.path.join(RAIZ, "modelos")
URDF_ATLAS = os.path.join(MODELOS, "atlas", "atlas_v4_with_multisense.urdf")
SDF_LAB = os.path.join(MODELOS, "botlab", "botlab.sdf")
URDF_CAJA = os.path.join(MODELOS, "boston_box.urdf")

POS_ATLAS = [-2.0, 3.0, -0.5]            # como en atlas.py
POS_CAJAS = ([-2.0, 3.0, -2.0], [0.0, 3.0, -2.0])
Z_CAJA = -1.5                            # cara superior de la caja

# ---------------- Grados de libertad que maneja la consola ----------------
BRAZO = ["shz", "shx", "ely", "elx", "wry", "wrx"]
NOMBRES_Q = (["torso_giro", "torso_adelante", "torso_lateral", "cabeza"]
             + [f"{l}_{j}" for l in ("der", "izq") for j in BRAZO] + ["sentadilla", "balanceo"])
IDX = {n: k for k, n in enumerate(NOMBRES_Q)}
ARTICULACION = {"torso_giro": "back_bkz", "torso_adelante": "back_bky", "torso_lateral": "back_bkx",
                "cabeza": "neck_ry"}
for _j in BRAZO:
    ARTICULACION[f"der_{_j}"] = f"r_arm_{_j}"
    ARTICULACION[f"izq_{_j}"] = f"l_arm_{_j}"

DESCRIPCION = {"torso_giro": "Torso giro", "torso_adelante": "Torso adelante", "torso_lateral": "Torso lateral",
               "cabeza": "Cabeza", "shz": "Hombro adelante (shz)", "shx": "Hombro arriba (shx)",
               "ely": "Brazo giro (ely)", "elx": "Codo (elx)", "wry": "Muneca giro (wry)",
               "wrx": "Muneca flexion (wrx)", "sentadilla": "Sentadilla", "balanceo": "Balanceo lateral"}
GRUPOS = [("TORSO Y CABEZA", ["torso_giro", "torso_adelante", "cabeza"]),
          ("HOMBRO", ["shz", "shx", "ely"]),
          ("CODO Y MUNECA", ["elx", "wry", "wrx"]),
          ("PIERNAS", ["sentadilla", "balanceo", "torso_lateral"])]
# el brazo izquierdo en espejo del derecho: shz, shx, elx y wrx cambian de signo
SIGNO_ESPEJO = {"shz": -1, "shx": -1, "ely": 1, "elx": -1, "wry": 1, "wrx": -1}
# rangos de uso (dentro de los límites del URDF) de lo que no es un brazo
RANGO = {"torso_giro": (-0.6, 0.6), "torso_adelante": (-0.2, 0.45), "torso_lateral": (-0.3, 0.3),
         "cabeza": (-0.6, 1.0), "sentadilla": (0.0, 0.7), "balanceo": (-0.1, 0.1)}
DER, IZQ, AMBOS = "DERECHO", "IZQUIERDO", "AMBOS (espejo)"

# ---------------- Movimiento ----------------
VMAX = {"torso": 0.6, "cabeza": 1.0, "brazo": 1.2, "sentadilla": 0.35, "balanceo": 0.1}
AMAX = {"torso": 1.5, "cabeza": 3.0, "brazo": 3.0, "sentadilla": 0.8, "balanceo": 0.3}
# control de equilibrio en los tobillos: corrección = Kp * error_CM + Kd * velocidad_CM
KP_EQ, KD_EQ = (1.5, -1.0), (0.3, -0.2)          # (adelante-atrás, lateral)
CORR_MAX = 0.35                                   # rad
ESPERA_POSE = 0.6
UMBRAL_CAPTURA = 90
MAX_POSES = 12
INVERTIR = (False, False, False)
PYB_HZ, CTRL_HZ = 240, 120

# ---------------- Poses ----------------
EN_CRUZ = {}                                                   # todo en cero = la imagen del taller
ABAJO = {"der_shx": 1.3, "izq_shx": -1.3, "der_elx": -0.3, "izq_elx": 0.3}
SALUDO = {"der_ely": 0.0, "der_elx": -1.5, "izq_shx": -1.3, "izq_elx": 0.3, "cabeza": 0.1, "torso_giro": -0.15}
AL_FRENTE = {"der_shz": 0.9, "izq_shz": -0.9, "der_shx": 0.3, "izq_shx": -0.3, "der_ely": 1.5, "izq_ely": 1.5}


def pose(d):
    q = np.zeros(len(NOMBRES_Q))
    for n, v in d.items():
        q[IDX[n]] = v
    return q


def demostraciones():
    saludo = [pose(SALUDO)]
    for _ in range(3):
        for d in (-0.4, 0.4):
            saludo.append(pose(SALUDO | {"der_elx": -1.5 + d}))
    saludo.append(pose(ABAJO))
    sentadilla = []
    for _ in range(2):
        sentadilla += [pose(AL_FRENTE | {"sentadilla": 0.6, "torso_adelante": 0.15}), pose(AL_FRENTE)]
    sentadilla.append(pose(ABAJO))
    giro = [pose(EN_CRUZ | {"torso_giro": 0.5, "cabeza": 0.2}), pose(EN_CRUZ | {"torso_giro": -0.5, "cabeza": 0.2}),
            pose(EN_CRUZ)]
    return [("SALUDO", saludo), ("SENTADILLA", sentadilla), ("GIRO DE TORSO", giro),
            ("BRAZOS ABAJO", [pose(ABAJO)]), ("EN CRUZ", [pose(EN_CRUZ)])]


# ==========================================================================
# Escena y robot
# ==========================================================================
def cargar_laboratorio(cliente):
    objs = p.loadSDF(SDF_LAB, globalScaling=2.0, physicsClientId=cliente)
    y_a_z = p.getQuaternionFromEuler([math.pi / 2, 0, math.pi / 2])      # el modelo viene con Y hacia arriba
    for o in objs:
        pos, orn = p.getBasePositionAndOrientation(o, physicsClientId=cliente)
        pos, orn = p.multiplyTransforms([0, 0, 0], y_a_z, pos, orn)
        p.resetBasePositionAndOrientation(o, pos, orn, physicsClientId=cliente)
    for pc in POS_CAJAS:
        p.loadURDF(URDF_CAJA, pc, useFixedBase=True, physicsClientId=cliente)


class Atlas:
    def __init__(self, cliente):
        self.c = cliente
        self.id = p.loadURDF(URDF_ATLAS, POS_ATLAS, physicsClientId=cliente)
        n = p.getNumJoints(self.id, physicsClientId=cliente)
        self.info = [p.getJointInfo(self.id, j, physicsClientId=cliente) for j in range(n)]
        self.j = {i[1].decode(): i[0] for i in self.info}
        self.moviles = [i[0] for i in self.info if i[2] != p.JOINT_FIXED]
        self.masas = [p.getDynamicsInfo(self.id, l, physicsClientId=cliente)[0] for l in range(-1, n)]
        self.pies = (self.j["l_leg_akx"], self.j["r_leg_akx"])
        # límites de cada grado de libertad de la consola
        self.bajo, self.alto = np.zeros(len(NOMBRES_Q)), np.zeros(len(NOMBRES_Q))
        self.vmax, self.amax = np.zeros(len(NOMBRES_Q)), np.zeros(len(NOMBRES_Q))
        for k, nombre in enumerate(NOMBRES_Q):
            if nombre in RANGO:
                lo, hi = RANGO[nombre]
            else:
                inf = self.info[self.j[ARTICULACION[nombre]]]
                lo, hi = inf[8], inf[9]
            tipo = ("brazo" if nombre[:4] in ("der_", "izq_") else "cabeza" if nombre == "cabeza"
                    else "torso" if nombre.startswith("torso") else nombre)
            self.bajo[k], self.alto[k] = lo, hi
            self.vmax[k], self.amax[k] = VMAX[tipo], AMAX[tipo]
        # eslabones de cada grado de libertad (para resaltarlos)
        hijos = {}
        for i in self.info:
            hijos.setdefault(i[16], []).append(i[0])

        def fijos(link):
            out = [link]
            for h in hijos.get(link, []):
                if self.info[h][2] == p.JOINT_FIXED:
                    out += fijos(h)
            return out
        piernas = [self.j[f"{s}_leg_{a}"] for s in "lr" for a in ("hpz", "hpx", "hpy", "kny", "aky", "akx")]
        self.eslabones = {}
        for nombre in NOMBRES_Q:
            juntas = piernas if nombre in ("sentadilla", "balanceo") else [self.j[ARTICULACION[nombre]]]
            links = []
            for jj in juntas:
                links += fijos(jj)
            self.eslabones[nombre] = sorted(set(links))
        # color y textura originales de cada eslabón (el Atlas tiene texturas: para
        # resaltarlo en amarillo hay que quitarle la textura y luego devolvérsela)
        self.colores, self.texturas = {}, {}
        for d in p.getVisualShapeData(self.id, flags=p.VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS,
                                      physicsClientId=cliente):
            self.colores.setdefault(d[1], d[7])
            self.texturas.setdefault(d[1], d[-1])
        self.resaltados = set()
        self.reponer()

    def reponer(self):
        p.resetBasePositionAndOrientation(self.id, POS_ATLAS, [0, 0, 0, 1], physicsClientId=self.c)
        p.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0], physicsClientId=self.c)
        for j in self.moviles:
            p.resetJointState(self.id, j, 0.0, 0.0, physicsClientId=self.c)

    def angulos(self, q, corr=(0.0, 0.0)):
        """Grados de libertad de la consola -> ángulo de cada articulación del robot."""
        a = {ARTICULACION[n]: q[IDX[n]] for n in ARTICULACION}
        s, b = q[IDX["sentadilla"]], q[IDX["balanceo"]]
        for lado in "lr":
            # sentadilla: cadera -s, rodilla 2s, tobillo -s -> los pies quedan planos y el torso vertical
            a[f"{lado}_leg_hpy"], a[f"{lado}_leg_kny"], a[f"{lado}_leg_aky"] = -s, 2 * s, -s + corr[0]
            # balanceo lateral: la cadera y el tobillo giran opuestos (paralelogramo)
            a[f"{lado}_leg_hpx"], a[f"{lado}_leg_akx"] = b, -b + corr[1]
        return a

    def mandar(self, q, corr):
        objetivo = self.angulos(q, corr)
        for j in self.moviles:
            inf = self.info[j]
            p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, targetPosition=objetivo.get(inf[1].decode(), 0.0),
                                    force=inf[10], maxVelocity=inf[11], physicsClientId=self.c)

    def leer(self):
        est = {i[1].decode(): p.getJointState(self.id, i[0], physicsClientId=self.c)[0]
               for i in self.info if i[2] != p.JOINT_FIXED}
        q = np.zeros(len(NOMBRES_Q))
        for n, art in ARTICULACION.items():
            q[IDX[n]] = est[art]
        q[IDX["sentadilla"]] = (est["l_leg_kny"] + est["r_leg_kny"]) / 4
        q[IDX["balanceo"]] = (est["l_leg_hpx"] + est["r_leg_hpx"]) / 2
        return q

    def centro_de_masa(self):
        pos, vel = np.zeros(3), np.zeros(3)
        for k, l in enumerate(range(-1, len(self.info))):
            if l == -1:
                x = p.getBasePositionAndOrientation(self.id, physicsClientId=self.c)[0]
                v = p.getBaseVelocity(self.id, physicsClientId=self.c)[0]
            else:
                s = p.getLinkState(self.id, l, computeLinkVelocity=1, physicsClientId=self.c)
                x, v = s[0], s[6]
            pos += self.masas[k] * np.array(x)
            vel += self.masas[k] * np.array(v)
        m = sum(self.masas)
        return pos / m, vel / m

    def apoyo(self):
        """Rectángulo que cubre los dos pies (x_min, x_max, y_min, y_max) y su centro."""
        cajas = [p.getAABB(self.id, pie, physicsClientId=self.c) for pie in self.pies]
        x0 = min(c[0][0] for c in cajas); x1 = max(c[1][0] for c in cajas)
        y0 = min(c[0][1] for c in cajas); y1 = max(c[1][1] for c in cajas)
        return (x0, x1, y0, y1), np.array([(x0 + x1) / 2, (y0 + y1) / 2])

    def orientacion(self):
        pos, orn = p.getBasePositionAndOrientation(self.id, physicsClientId=self.c)
        return np.array(pos), p.getEulerFromQuaternion(orn)

    def resaltar(self, nombres):
        links = set()
        for n in nombres:
            links |= set(self.eslabones[n])
        if links == self.resaltados:
            return
        for l in self.resaltados - links:
            if l not in self.colores:
                continue
            p.changeVisualShape(self.id, l, rgbaColor=self.colores[l], textureUniqueId=self.texturas[l],
                                physicsClientId=self.c)
        for l in links - self.resaltados:
            if l not in self.colores:
                continue
            p.changeVisualShape(self.id, l, rgbaColor=[1.0, 0.82, 0.1, 1], textureUniqueId=-1, physicsClientId=self.c)
        self.resaltados = links


# ==========================================================================
# Movimiento fluido (mismas ideas que en el inciso b)
# ==========================================================================
class Suavizador:
    """Cada grado de libertad sigue a su objetivo con velocidad y aceleración limitadas."""

    def __init__(self, q, vmax, amax):
        self.q, self.v = np.array(q, dtype=float), np.zeros(len(q))
        self.vmax, self.amax = vmax, amax

    def paso(self, objetivo, dt):
        e = objetivo - self.q
        v_des = np.sign(e) * np.minimum.reduce([self.vmax, np.sqrt(2 * self.amax * np.abs(e)), np.abs(e) / dt])
        self.v += np.clip(v_des - self.v, -self.amax * dt, self.amax * dt)
        self.q = self.q + self.v * dt
        return self.q


class MinimoJerk:
    """q(t) = q0 + Δq (10τ³ - 15τ⁴ + 6τ⁵): todos los grados de libertad salen y llegan
    juntos, sin tirones; la duración respeta velocidad (1.875 Δq/T) y aceleración (5.77 Δq/T²)."""

    def __init__(self, q0, q1, vmax, amax):
        self.q0, self.dq = np.array(q0, dtype=float), np.array(q1, dtype=float) - q0
        d = np.abs(self.dq)
        self.T = max(0.5, float(np.max(np.maximum(1.875 * d / vmax, np.sqrt(5.77 * d / amax)))))

    def en(self, t):
        tau = min(max(t / self.T, 0.0), 1.0)
        s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
        ds = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) / self.T
        return self.q0 + self.dq * s, self.dq * ds


def barra(valor, bajo, alto, ancho=22):
    k = int(round((valor - bajo) / (alto - bajo) * (ancho - 1)))
    k = min(max(k, 0), ancho - 1)
    return "[" + "-" * k + "#" + "-" * (ancho - 1 - k) + "]"


# ==========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--sin-serial", action="store_true", help="usar la consola virtual (sliders)")
    ap.add_argument("--sin-ventana", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--duracion", type=float, default=0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    for f in (URDF_ATLAS, SDF_LAB, URDF_CAJA):
        if not os.path.exists(f):
            print(f"No encuentro el archivo:\n  {f}")
            return

    marca = time.strftime("%Y%m%d_%H%M%S")
    carpeta_logs = os.path.join(AQUI, "logs")
    os.makedirs(carpeta_logs, exist_ok=True)

    consola = None
    if not args.sin_serial:
        try:
            consola = ConsolaESP32(args.port, archivo_tramas=os.path.join(carpeta_logs, f"tramas_{marca}.csv"))
        except Exception as e:
            print(f"No se pudo abrir {args.port}: {e}")
            print("Cierra el Serial Monitor de PlatformIO y revisa el puerto.")
            return
        print(f"Puerto {args.port} abierto")

    # ---- Simulación ----
    cliente = p.connect(p.DIRECT if args.sin_ventana else p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=cliente)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=cliente)
    p.setGravity(0, 0, -9.81, physicsClientId=cliente)
    p.setTimeStep(1.0 / PYB_HZ, physicsClientId=cliente)
    print("Cargando el laboratorio...")
    cargar_laboratorio(cliente)
    robot = Atlas(cliente)
    q_inicio = pose(EN_CRUZ)
    robot.mandar(q_inicio, (0.0, 0.0))
    for _ in range(PYB_HZ):                       # 1 s para que se asiente sobre la caja
        p.stepSimulation(physicsClientId=cliente)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=cliente)
    if not args.sin_ventana:
        p.resetDebugVisualizerCamera(cameraDistance=2.4 if args.sin_serial else 2.0, cameraYaw=65,
                                     cameraPitch=-6, cameraTargetPosition=[-2.0, 3.0, -0.38],
                                     physicsClientId=cliente)
    # marcador del centro de masa proyectado sobre la caja
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.025, length=0.004, rgbaColor=[0.1, 0.7, 0.2, 1],
                              physicsClientId=cliente)
    marcador_cm = p.createMultiBody(0, -1, vis, [POS_ATLAS[0], POS_ATLAS[1], Z_CAJA + 0.003],
                                    physicsClientId=cliente)
    lineas_apoyo = [-1] * 4

    if consola is not None:
        consola.start()
        print("Consola ESP32 conectada")
    else:
        consola = ConsolaVirtual(["Grado 1", "Grado 2", "Grado 3"],
                                 ["Siguiente grupo", "Cambiar brazo", "Grabar pose",
                                  "Reproducir/detener", "Demostracion"],
                                 valores_iniciales=(2048, 2048, 2048), cliente=cliente)
    # en el laboratorio hay objetos entre la cámara y el robot: el texto va cerca de la cámara
    hud = Hud(8, cliente, x_ndc=-0.48, y_ndc=0.84, cerca=0.3) if args.sin_serial else Hud(8, cliente, cerca=0.3)

    # ---- Registro ----
    f_log = open(os.path.join(carpeta_logs, f"atlas_{marca}.csv"), "w", newline="", encoding="utf-8")
    log = csv.writer(f_log)
    log.writerow(["t_s", "modo", "brazo", "grupo", "evento"]
                 + [f"{n}_obj" for n in NOMBRES_Q] + [f"{n}_suave" for n in NOMBRES_Q]
                 + [f"{n}_real" for n in NOMBRES_Q]
                 + ["cm_x", "cm_y", "cm_z", "apoyo_cx", "apoyo_cy", "margen_m", "corr_adelante", "corr_lateral",
                    "inclinacion_grados"])

    # ---- Estado ----
    dt = 1.0 / CTRL_HZ
    sub = PYB_HZ // CTRL_HZ
    suave = Suavizador(q_inicio, robot.vmax, robot.amax)
    objetivo = q_inicio.copy()
    grupo, brazo = 0, DER
    capturado, pots_ref = [False] * 3, None
    grabadas = []
    secuencia, nombre_sec, en_bucle = [], "", False
    tramo, t_tramo, paso_sec, esperando = None, 0.0, 0, None
    lista_demos, n_demo = demostraciones(), 0
    t_boton3, t_caida, caidas = None, None, 0
    i, t_hud = 0, -1.0
    inicio_real = time.time()
    print("Listo. 1 = grupo | 2 = brazo | 3 = grabar (mantener: borrar) | 4 = reproducir | 5 = demostración")

    def evento_txt(txt):
        print(f"[{i * dt:7.2f}s] {txt}")
        return txt

    def detener():
        nonlocal secuencia, tramo, esperando, objetivo
        secuencia, tramo, esperando = [], None, None
        objetivo = suave.q.copy()
        suave.v[:] = 0.0

    def iniciar(poses, nombre, bucle):
        nonlocal secuencia, nombre_sec, en_bucle, tramo, t_tramo, paso_sec, esperando, capturado
        secuencia, nombre_sec, en_bucle = [np.array(q) for q in poses], nombre, bucle
        paso_sec, esperando = 0, None
        tramo, t_tramo = MinimoJerk(suave.q, secuencia[0], robot.vmax, robot.amax), t
        capturado = [False] * 3

    def nombres_grupo():
        out = []
        for g in GRUPOS[grupo][1]:
            if g in BRAZO:
                lados = ["der", "izq"] if brazo == AMBOS else (["der"] if brazo == DER else ["izq"])
                out += [f"{l}_{g}" for l in lados]
            else:
                out.append(g)
        return out

    def nombre_principal(g):
        return f"{'izq' if brazo == IZQ else 'der'}_{g}" if g in BRAZO else g

    try:
        while True:
            t = i * dt
            if args.duracion and t > args.duracion:
                break
            evento = ""
            m = consola.leer()
            pots = m["pots"] if (m and consola.conectado()) else None
            if pots is not None:
                pots = [4095 - a if inv else a for a, inv in zip(pots, INVERTIR)]

            # ---- 1) Pulsadores ----
            for btn in consola.presionados():
                if btn == 1:
                    grupo = (grupo + 1) % len(GRUPOS)
                    evento = evento_txt(f"boton 1: grupo {grupo + 1} {GRUPOS[grupo][0]}")
                elif btn == 2:
                    brazo = {DER: IZQ, IZQ: AMBOS, AMBOS: DER}[brazo]
                    evento = evento_txt(f"boton 2: brazo {brazo}")
                elif btn == 3:
                    t_boton3 = t
                    if len(grabadas) < MAX_POSES:
                        grabadas.append(suave.q.copy())
                        evento = evento_txt(f"boton 3: pose {len(grabadas)} grabada")
                    else:
                        evento = evento_txt(f"boton 3: ya hay {MAX_POSES} poses grabadas")
                elif btn == 4:
                    if secuencia:
                        detener()
                        evento = evento_txt("boton 4: reproducción detenida")
                    elif grabadas:
                        iniciar(grabadas, "POSES GRABADAS", True)
                        evento = evento_txt(f"boton 4: reproduciendo {len(grabadas)} poses en bucle")
                    else:
                        evento = evento_txt("boton 4: no hay poses grabadas (usa el pulsador 3)")
                elif btn == 5:
                    nombre, poses_demo = lista_demos[n_demo]
                    n_demo = (n_demo + 1) % len(lista_demos)
                    iniciar(poses_demo, nombre, False)
                    evento = evento_txt(f"boton 5: demostración {nombre}")
                if btn in (1, 2) or (btn in (4, 5) and not secuencia):
                    capturado, pots_ref = [False] * 3, pots

            teclas = p.getKeyboardEvents(physicsClientId=cliente) if not args.sin_ventana else {}
            mantenido = t_boton3 is not None and consola.mantenido(3) and t - t_boton3 >= 1.5
            if mantenido or (ord("c") in teclas and teclas[ord("c")] & p.KEY_WAS_TRIGGERED):
                t_boton3 = None
                grabadas = []
                if secuencia and nombre_sec == "POSES GRABADAS":
                    detener()
                evento = evento_txt("poses grabadas borradas")
            if t_boton3 is not None and not consola.mantenido(3) and t - t_boton3 > 0.1:
                t_boton3 = None

            # ---- 2) Caída y reposición ----
            base, (rx, ry, _) = robot.orientacion()
            inclinacion = math.degrees(max(abs(rx), abs(ry)))
            if t_caida is None and (inclinacion > 35 or base[2] < Z_CAJA + 0.5):
                t_caida, caidas = t, caidas + 1
                detener()
                evento = evento_txt(f"¡el Atlas se cayó! (caída {caidas}); se repone en 2.5 s")
            reponer = t_caida is not None and t - t_caida >= 2.5
            if reponer or (ord("r") in teclas and teclas[ord("r")] & p.KEY_WAS_TRIGGERED):
                detener()
                robot.reponer()
                suave = Suavizador(q_inicio, robot.vmax, robot.amax)
                objetivo, t_caida = q_inicio.copy(), None
                capturado, pots_ref = [False] * 3, pots
                evento = evento_txt("Atlas repuesto de pie sobre la caja")

            # ---- 3) Objetivo ----
            if secuencia:
                if esperando is None:
                    q_tr, v_tr = tramo.en(t - t_tramo)
                    suave.q, suave.v = q_tr.copy(), v_tr.copy()
                    objetivo = secuencia[paso_sec].copy()
                    if t - t_tramo >= tramo.T:
                        esperando = t
                elif t - esperando >= ESPERA_POSE:
                    err = math.degrees(float(np.max(np.abs(robot.leer()[:16] - secuencia[paso_sec][:16]))))
                    evento = evento_txt(f"{nombre_sec}: pose {paso_sec + 1}/{len(secuencia)} alcanzada "
                                        f"(error máx {err:.2f} grados)")
                    paso_sec += 1
                    if paso_sec >= len(secuencia):
                        if en_bucle:
                            paso_sec = 0
                        else:
                            evento_txt(f"{nombre_sec}: terminada")
                            detener()
                            pots_ref = pots
                    if secuencia:
                        tramo, t_tramo, esperando = MinimoJerk(suave.q, secuencia[paso_sec], robot.vmax,
                                                               robot.amax), t, None
            elif pots is not None and t_caida is None:
                if pots_ref is None:
                    pots_ref = pots
                for k in range(3):
                    if not capturado[k] and abs(pots[k] - pots_ref[k]) > UMBRAL_CAPTURA:
                        capturado[k] = True
                    if not capturado[k]:
                        continue
                    g = GRUPOS[grupo][1][k]
                    n = IDX[nombre_principal(g)]
                    objetivo[n] = adc_a_rango(pots[k], robot.bajo[n], robot.alto[n])
                    if g in BRAZO and brazo == AMBOS:
                        n2 = IDX[f"izq_{g}"]
                        objetivo[n2] = min(max(SIGNO_ESPEJO[g] * objetivo[n], robot.bajo[n2]), robot.alto[n2])

            if not secuencia:
                suave.paso(objetivo, dt)

            # ---- 4) Control de equilibrio (estrategia de tobillo) ----
            cm, v_cm = robot.centro_de_masa()
            (x0, x1, y0, y1), centro = robot.apoyo()
            yaw = robot.orientacion()[1][2]
            rot = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])  # mundo -> robot
            e = rot @ (cm[:2] - centro)
            ve = rot @ v_cm[:2]
            corr = (float(np.clip(KP_EQ[0] * e[0] + KD_EQ[0] * ve[0], -CORR_MAX, CORR_MAX)),
                    float(np.clip(KP_EQ[1] * e[1] + KD_EQ[1] * ve[1], -CORR_MAX, CORR_MAX)))
            if t_caida is not None:
                corr = (0.0, 0.0)
            margen = min(cm[0] - x0, x1 - cm[0], cm[1] - y0, y1 - cm[1])

            robot.mandar(suave.q, corr)
            for _ in range(sub):
                p.stepSimulation(physicsClientId=cliente)
            real = robot.leer()

            # ---- 5) Registro ----
            modo = ("DEMO " + nombre_sec if not en_bucle else "REPRODUCIENDO") if secuencia else \
                ("CAIDO" if t_caida is not None else "MANUAL")
            log.writerow([f"{t:.3f}", modo, brazo, grupo + 1, evento]
                         + [f"{v:.4f}" for v in objetivo] + [f"{v:.4f}" for v in suave.q]
                         + [f"{v:.4f}" for v in real]
                         + [f"{v:.4f}" for v in cm] + [f"{centro[0]:.4f}", f"{centro[1]:.4f}", f"{margen:.4f}",
                                                       f"{corr[0]:.4f}", f"{corr[1]:.4f}", f"{inclinacion:.2f}"])

            # ---- 6) Pantalla ----
            if not args.sin_ventana:
                robot.resaltar([] if (secuencia or t_caida is not None) else nombres_grupo())
                if t - t_hud >= 0.1:
                    color_m = [0.1, 0.7, 0.2] if margen > 0.03 else ([1.0, 0.6, 0.0] if margen > 0 else [0.9, 0.1, 0.1])
                    p.resetBasePositionAndOrientation(marcador_cm, [cm[0], cm[1], Z_CAJA + 0.003], [0, 0, 0, 1],
                                                      physicsClientId=cliente)
                    p.changeVisualShape(marcador_cm, -1, rgbaColor=color_m + [1], physicsClientId=cliente)
                    esquinas = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                    for k in range(4):
                        a_, b_ = esquinas[k], esquinas[(k + 1) % 4]
                        lineas_apoyo[k] = p.addUserDebugLine([a_[0], a_[1], Z_CAJA + 0.004], [b_[0], b_[1], Z_CAJA + 0.004],
                                                             [0.1, 0.4, 0.9], 2, replaceItemUniqueId=lineas_apoyo[k],
                                                             physicsClientId=cliente)
                if t - t_hud >= 0.2:
                    t_hud = t
                    if t_caida is not None:
                        linea1, color1 = "EL ATLAS SE CAYO - reponiendo...", [0.9, 0.1, 0.1]
                    elif secuencia:
                        linea1 = (f"{'REPRODUCIENDO' if en_bucle else 'DEMOSTRACION'}: {nombre_sec}  "
                                  f"pose {paso_sec + 1}/{len(secuencia)}   (4 = detener)")
                        color1 = [0.6, 0.3, 0.0]
                    else:
                        linea1 = f"Grupo {grupo + 1}/4: {GRUPOS[grupo][0]}"
                        if any(g in BRAZO for g in GRUPOS[grupo][1]):
                            linea1 += f"  |  Brazo: {brazo}"
                        color1 = [0.1, 0.3, 0.7]
                    lineas = [(linea1, color1)]
                    for k, g in enumerate(GRUPOS[grupo][1]):
                        n = IDX[nombre_principal(g)]
                        aviso = "" if (capturado[k] or secuencia) else "  (mueve el pot)"
                        lineas.append((f"Pot {k + 1}  {DESCRIPCION[g]:<22} {math.degrees(real[n]):+6.1f} gr "
                                       f"{barra(real[n], robot.bajo[n], robot.alto[n])}{aviso}", [0.15, 0.15, 0.15]))
                    estado_eq = "ESTABLE" if margen > 0.03 else ("AL LIMITE" if margen > 0 else "FUERA DE LOS PIES")
                    lineas.append((f"Equilibrio: {estado_eq}  (margen {margen * 100:+.1f} cm, inclinacion "
                                   f"{inclinacion:.1f} gr)", color_m))
                    lineas.append((f"Poses grabadas: {len(grabadas)}  |  caidas: {caidas}", [0.1, 0.5, 0.2]))
                    lineas.append((consola.texto_estado(), [0, 0.45, 0] if consola.conectado() else [0.8, 0, 0]))
                    lineas.append(("1=Grupo 2=Brazo 3=Grabar (mantener: borrar) 4=Reproducir 5=Demo  R=reponer",
                                   [0.35, 0.35, 0.35]))
                    hud.actualizar(lineas)

            espera = inicio_real + (i + 1) * dt - time.time()
            if espera > 0:
                time.sleep(espera)
            i += 1
    except (KeyboardInterrupt, p.error):
        pass
    finally:
        f_log.close()
        consola.cerrar()
        try:
            p.disconnect(physicsClientId=cliente)
        except p.error:
            pass
        print(f"Registros guardados en: {carpeta_logs}")


if __name__ == "__main__":
    main()
