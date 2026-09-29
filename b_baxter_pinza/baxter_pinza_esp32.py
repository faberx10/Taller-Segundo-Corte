"""
Taller Segundo Corte - Inciso (b): consola ESP32 para el robot Baxter con pinza
Micros y Laboratorio - UMNG

El Baxter (modelo toms_baxter.urdf de erwincoumans/pybullet_robots) mueve sus dos
brazos por cinemática inversa, como en baxter_ik_demo.py, pero en vez de
"teletransportar" las articulaciones las mueve con sus motores, límites de
fuerza y de velocidad reales. Los potenciómetros indican A DÓNDE va la pinza
y el programa genera una trayectoria suave hasta ahí. Con la pinza se pueden
coger los bloques de la mesa y dejarlos en la bandeja.

    Pot 1 -> X  (cerca / lejos del robot)       Pulsador 1 -> cambiar brazo activo (derecho / izquierdo)
    Pot 2 -> Y  (derecha / izquierda)           Pulsador 2 -> abrir / cerrar la pinza del brazo activo
    Pot 3 -> Z  (altura sobre la mesa)          Pulsador 3 -> modo espejo (los dos brazos a la vez)
                                                Pulsador 4 -> posición inicial (mantener 1.5 s: reiniciar bloques)
                                                Pulsador 5 -> rutina automática: llevar los bloques a la bandeja
    Tecla R en la ventana -> reiniciar los bloques.

Mientras la pinza está cerrada sobre un bloque, una sujeción asistida lo mantiene
fijo a ella (ver Baxter.actualizar_sujecion) y se suelta al abrir la pinza.

Uso:
    python baxter_pinza_esp32.py                 # consola ESP32 en COM5
    python baxter_pinza_esp32.py --port COM7
    python baxter_pinza_esp32.py --sin-serial    # consola virtual (sliders)
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)
sys.path.insert(0, RAIZ)
from comun.consola import ConsolaESP32, ConsolaVirtual, Hud, adc_a_rango  # noqa: E402

URDF_BAXTER = os.path.join(RAIZ, "modelos", "baxter_common", "baxter_description", "urdf",
                           "toms_baxter.urdf")

# ---------------- Escena (coordenadas del Baxter, en metros) ----------------
Z_PISO = -0.926                  # el origen del Baxter está sobre su pedestal
MESA = {"centro": (0.62, 0.0), "medio": (0.25, 0.62), "z_sup": -0.18}
TOP = MESA["z_sup"]
MEDIO_BLOQUE = (0.015, 0.015, 0.025)   # bloque de 3 x 3 x 5 cm (la pinza abre ~4.2 cm)
MASA_BLOQUE = 0.05
BLOQUES_INICIO = {"rojo": (0.60, -0.36), "azul": (0.60, 0.36)}
COLOR_BLOQUE = {"rojo": [0.85, 0.12, 0.12, 1], "azul": [0.12, 0.35, 0.9, 1]}
BANDEJA = {"centro": (0.66, 0.0), "medio": 0.08, "alto_pared": 0.03}

# ---------------- Brazos ----------------
BRAZOS = {
    "der": {"nombre": "DERECHO", "efector": 26, "dedos": (27, 29), "hombro": 12,
            "juntas": ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"],
            "inicio": (0.50, -0.42, TOP + 0.22), "rango_y": (-0.65, 0.08), "color": [0.85, 0.2, 0.2]},
    "izq": {"nombre": "IZQUIERDO", "efector": 48, "dedos": (49, 51), "hombro": 34,
            "juntas": ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"],
            "inicio": (0.50, 0.42, TOP + 0.22), "rango_y": (-0.08, 0.65), "color": [0.2, 0.35, 0.9]},
}
POSE_REPOSO = {"s0": 0.3, "s1": -0.6, "e0": 0.0, "e1": 1.6, "w0": 0.0, "w1": 0.6, "w2": 0.0}

INVERTIR = (False, False, False) # invertir el sentido de los pots 1, 2, 3
RANGO_X = (0.40, 0.80)           # pot 1
RANGO_Z = (TOP + 0.015, TOP + 0.33)   # pot 3 (el mínimo deja los dedos 1 cm sobre la mesa)
ALCANCE = (0.44, 0.72)           # distancia horizontal al hombro donde la IK es fiable
SEPARACION_Y = 0.14              # distancia mínima en Y entre las dos pinzas

DEDO_ABIERTO = 0.021             # m por dedo (abertura total ~4.2 cm)
T_SUJECION = 0.25                # s que los dos dedos deben apretar el bloque antes de fijarlo
FUERZA_SUJECION = 40.0           # N máximos de la sujeción asistida
FUERZA_DEDO, VEL_DEDO = 20.0, 0.06

VMAX_MANUAL, AMAX = 0.25, 0.8    # m/s, m/s²   perfil de la trayectoria cartesiana
VMAX_RUTINA = 0.22
UMBRAL_CAPTURA = 90              # cuentas de ADC para "tomar el control" con los pots
PYB_HZ, CTRL_HZ = 240, 120

MANUAL, ESPEJO, RUTINA = "MANUAL", "ESPEJO", "RUTINA"


# ==========================================================================
# Trayectoria suave: limita velocidad y aceleración del objetivo cartesiano
# ==========================================================================
class Suavizador:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.vel = np.zeros(3)

    def paso(self, objetivo, dt, vmax, amax):
        error = np.asarray(objetivo, dtype=float) - self.pos
        dist = float(np.linalg.norm(error))
        if dist < 1e-5 and np.linalg.norm(self.vel) < 1e-3:
            self.pos, self.vel = np.array(objetivo, dtype=float), np.zeros(3)
            return self.pos
        # velocidad deseada: hacia el objetivo, frenando a tiempo (v = sqrt(2 a d))
        rapidez = min(vmax, math.sqrt(2 * amax * dist), dist / dt)
        v_deseada = error / max(dist, 1e-9) * rapidez
        dv = v_deseada - self.vel
        dv_max = amax * dt
        n = float(np.linalg.norm(dv))
        if n > dv_max:
            dv *= dv_max / n
        self.vel += dv
        self.pos = self.pos + self.vel * dt
        return self.pos

    def llego(self, objetivo, tol=0.002):
        return np.linalg.norm(np.asarray(objetivo) - self.pos) < tol and np.linalg.norm(self.vel) < 0.01


# ==========================================================================
# Robot
# ==========================================================================
class Baxter:
    def __init__(self, cliente):
        self.c = cliente
        self.id = p.loadURDF(URDF_BAXTER, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                             physicsClientId=cliente)
        self.dof, self.info = [], {}
        for j in range(p.getNumJoints(self.id, physicsClientId=cliente)):
            inf = p.getJointInfo(self.id, j, physicsClientId=cliente)
            if inf[3] > -1:
                self.dof.append(j)
                self.info[j] = {"nombre": inf[1].decode(), "min": inf[8], "max": inf[9],
                                "fuerza": inf[10], "vel": inf[11]}
        self.pos_en_ik = {j: k for k, j in enumerate(self.dof)}       # índice en la solución de IK
        self.por_nombre = {v["nombre"]: j for j, v in self.info.items()}
        # límites para la IK tomados del propio URDF (baxter_ik_demo usaba ±2 fijos)
        self.ll = [self.info[j]["min"] for j in self.dof]
        self.ul = [self.info[j]["max"] for j in self.dof]
        self.jr = [u - l for l, u in zip(self.ll, self.ul)]
        self.reposo = [0.0] * len(self.dof)
        for lado, b in BRAZOS.items():
            signo = 1 if lado == "der" else -1
            for n in b["juntas"]:
                corto = n.split("_")[1]
                valor = POSE_REPOSO[corto] * (signo if corto in ("s0", "e0", "w0", "w2") else 1)
                self.reposo[self.pos_en_ik[self.por_nombre[n]]] = valor
        for j in self.dof:
            p.resetJointState(self.id, j, self.reposo[self.pos_en_ik[j]], physicsClientId=cliente)
        # fricción alta en los dedos para que el agarre sea por contacto real
        for lado in BRAZOS.values():
            for d in lado["dedos"]:
                for link in (d, d + 1):
                    p.changeDynamics(self.id, link, lateralFriction=2.0, spinningFriction=0.01,
                                     physicsClientId=cliente)
        self.objetivo_q = {j: self.reposo[self.pos_en_ik[j]] for j in self.dof}
        self.sujecion = {l: None for l in BRAZOS}       # (id de la restricción, nombre del bloque)
        self._t_contacto = {l: 0.0 for l in BRAZOS}

    def efector(self, lado):
        s = p.getLinkState(self.id, BRAZOS[lado]["efector"], computeForwardKinematics=True,
                           physicsClientId=self.c)
        return np.array(s[4]), s[5]

    def hombro(self, lado):
        return np.array(p.getLinkState(self.id, BRAZOS[lado]["hombro"], computeForwardKinematics=True,
                                       physicsClientId=self.c)[4])

    def ik(self, lado, pos, yaw=0.0):
        orient = p.getQuaternionFromEuler([math.pi, 0.0, yaw])      # pinza apuntando hacia abajo
        q = p.calculateInverseKinematics(self.id, BRAZOS[lado]["efector"], list(pos), orient,
                                         lowerLimits=self.ll, upperLimits=self.ul, jointRanges=self.jr,
                                         restPoses=self.reposo, maxNumIterations=60, residualThreshold=1e-4,
                                         physicsClientId=self.c)
        for n in BRAZOS[lado]["juntas"]:
            j = self.por_nombre[n]
            self.objetivo_q[j] = q[self.pos_en_ik[j]]

    def colocar(self, lado, pos, yaw=0.0):
        """Pone el brazo directamente en 'pos' (solo al iniciar)."""
        for _ in range(20):
            self.ik(lado, pos, yaw)
            for n in BRAZOS[lado]["juntas"]:
                j = self.por_nombre[n]
                p.resetJointState(self.id, j, self.objetivo_q[j], physicsClientId=self.c)

    def error_ik(self, lado, puntos, yaw):
        """Prueba (sin mover el robot) si la IK alcanza 'puntos' con ese giro de muñeca.
        Devuelve el peor error: metros de posición + 5 cm por cada radián de inclinación/giro."""
        estado = p.saveState(physicsClientId=self.c)
        guardado = dict(self.objetivo_q)
        peor = 0.0
        for pos in puntos:
            for _ in range(15):
                self.ik(lado, pos, yaw)
                for n in BRAZOS[lado]["juntas"]:
                    j = self.por_nombre[n]
                    p.resetJointState(self.id, j, self.objetivo_q[j], physicsClientId=self.c)
            real, orient = self.efector(lado)
            dif = p.getDifferenceQuaternion(orient, p.getQuaternionFromEuler([math.pi, 0.0, yaw]))
            angulo = 2 * math.acos(min(1.0, abs(dif[3])))
            peor = max(peor, float(np.linalg.norm(real - np.asarray(pos))) + 0.05 * angulo)
        p.restoreState(estado, physicsClientId=self.c)
        p.removeState(estado, physicsClientId=self.c)
        self.objetivo_q = guardado
        return peor

    def actualizar_sujecion(self, dedos, bloques, dt):
        """Sujeción asistida (como el plugin 'grasp fix' de Gazebo).

        La fricción de Bullet entre dos dedos rígidos y un bloque deja que el bloque
        se deslice o gire al mover el brazo rápido, cosa que en el robot real evita
        el caucho de los dedos. Por eso, cuando la pinza está CERRADA y los DOS dedos
        llevan T_SUJECION segundos apretando el mismo bloque, se une el bloque a la
        pinza con una restricción de fuerza limitada. Al abrir la pinza se suelta.
        Devuelve un texto si algo cambió."""
        cambios = []
        for lado in BRAZOS:
            actual = self.sujecion[lado]
            if dedos[lado]:                               # pinza abierta -> soltar
                self._t_contacto[lado] = 0.0
                if actual:
                    p.removeConstraint(actual[0], physicsClientId=self.c)
                    self.sujecion[lado] = None
                    cambios.append(f"{lado}: suelta el bloque {actual[1]}")
                continue
            if actual:
                continue
            tocado = next((n for n, b in bloques.items() if self.sostiene(lado, b)), None)
            self._t_contacto[lado] = self._t_contacto[lado] + dt if tocado else 0.0
            if tocado and self._t_contacto[lado] >= T_SUJECION:
                ef = BRAZOS[lado]["efector"]
                pe, oe = self.efector(lado)
                pb, ob = p.getBasePositionAndOrientation(bloques[tocado], physicsClientId=self.c)
                inv_p, inv_o = p.invertTransform(pe, oe)
                rel_p, rel_o = p.multiplyTransforms(inv_p, inv_o, pb, ob)
                cid = p.createConstraint(self.id, ef, bloques[tocado], -1, p.JOINT_FIXED, [0, 0, 0],
                                         rel_p, [0, 0, 0], rel_o, physicsClientId=self.c)
                p.changeConstraint(cid, maxForce=FUERZA_SUJECION, physicsClientId=self.c)
                self.sujecion[lado] = (cid, tocado)
                cambios.append(f"{lado}: sujeta el bloque {tocado}")
        return "; ".join(cambios)

    def soltar_todo(self):
        for lado, actual in self.sujecion.items():
            if actual:
                p.removeConstraint(actual[0], physicsClientId=self.c)
                self.sujecion[lado] = None

    def aplicar(self, dedos):
        """Manda las posiciones objetivo a los motores con los límites del URDF."""
        for lado, b in BRAZOS.items():
            for n in b["juntas"]:
                j = self.por_nombre[n]
                p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, targetPosition=self.objetivo_q[j],
                                        force=self.info[j]["fuerza"], maxVelocity=self.info[j]["vel"],
                                        positionGain=0.08, physicsClientId=self.c)
            dl, dr = b["dedos"]
            if dedos[lado]:     # abrir: posición
                for d, s in ((dl, 1), (dr, -1)):
                    p.setJointMotorControl2(self.id, d, p.POSITION_CONTROL, targetPosition=s * DEDO_ABIERTO,
                                            force=FUERZA_DEDO, maxVelocity=VEL_DEDO, physicsClientId=self.c)
            else:               # cerrar: velocidad con fuerza máxima -> aprieta con fuerza constante
                for d, s in ((dl, 1), (dr, -1)):
                    p.setJointMotorControl2(self.id, d, p.VELOCITY_CONTROL, targetVelocity=-s * VEL_DEDO,
                                            force=FUERZA_DEDO, physicsClientId=self.c)
        cabeza = self.por_nombre["head_pan"]
        p.setJointMotorControl2(self.id, cabeza, p.POSITION_CONTROL, targetPosition=0.0, force=20,
                                physicsClientId=self.c)

    def abertura(self, lado):
        dl, dr = BRAZOS[lado]["dedos"]
        return (p.getJointState(self.id, dl, physicsClientId=self.c)[0]
                - p.getJointState(self.id, dr, physicsClientId=self.c)[0])

    def sostiene(self, lado, bloque):
        """True si los DOS dedos de la pinza tocan el bloque."""
        dl, dr = BRAZOS[lado]["dedos"]
        toca = lambda d: any(c[3] in (d, d + 1) for c in p.getContactPoints(self.id, bloque, physicsClientId=self.c))
        return toca(dl) and toca(dr)


# ==========================================================================
# Escena
# ==========================================================================
def caja(cliente, medio, pos, rgba, masa=0.0, friccion=1.0, colision=True):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=medio, physicsClientId=cliente) if colision else -1
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=medio, rgbaColor=rgba, physicsClientId=cliente)
    cuerpo = p.createMultiBody(masa, col, vis, pos, physicsClientId=cliente)
    if colision:
        p.changeDynamics(cuerpo, -1, lateralFriction=friccion, physicsClientId=cliente)
    return cuerpo


def crear_escena(cliente):
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cliente)
    p.loadURDF("plane.urdf", [0, 0, Z_PISO], physicsClientId=cliente)
    cx, cy = MESA["centro"]
    mx, my = MESA["medio"]
    caja(cliente, [mx, my, 0.02], [cx, cy, TOP - 0.02], [0.72, 0.55, 0.38, 1])
    alto_pata = (TOP - 0.04 - Z_PISO) / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            caja(cliente, [0.025, 0.025, alto_pata], [cx + sx * (mx - 0.04), cy + sy * (my - 0.04), Z_PISO + alto_pata],
                 [0.45, 0.33, 0.22, 1])
    # bandeja: base + 4 paredes bajas
    bx, by = BANDEJA["centro"]
    m, h = BANDEJA["medio"], BANDEJA["alto_pared"]
    verde = [0.15, 0.6, 0.25, 1]
    caja(cliente, [m, m, 0.004], [bx, by, TOP + 0.004], verde)
    for dx, dy, hx, hy in ((m, 0, 0.004, m), (-m, 0, 0.004, m), (0, m, m, 0.004), (0, -m, m, 0.004)):
        caja(cliente, [hx, hy, h / 2], [bx + dx, by + dy, TOP + h / 2], verde)
    p.addUserDebugText("BANDEJA", [bx, by, TOP + 0.10], [0.1, 0.5, 0.2], textSize=1.2, physicsClientId=cliente)
    bloques = {}
    for nombre, (x, y) in BLOQUES_INICIO.items():
        bloques[nombre] = caja(cliente, list(MEDIO_BLOQUE), [x, y, TOP + MEDIO_BLOQUE[2]], COLOR_BLOQUE[nombre],
                             masa=MASA_BLOQUE, friccion=1.0)
    return bloques


def reiniciar_bloques(bloques, cliente):
    for nombre, (x, y) in BLOQUES_INICIO.items():
        p.resetBasePositionAndOrientation(bloques[nombre], [x, y, TOP + MEDIO_BLOQUE[2]], [0, 0, 0, 1],
                                          physicsClientId=cliente)
        p.resetBaseVelocity(bloques[nombre], [0, 0, 0], [0, 0, 0], physicsClientId=cliente)


def pose_bloque(bloque, cliente):
    pos, orn = p.getBasePositionAndOrientation(bloque, physicsClientId=cliente)
    return np.array(pos), p.getEulerFromQuaternion(orn)[2]


def en_bandeja(pos):
    bx, by = BANDEJA["centro"]
    m = BANDEJA["medio"]
    return abs(pos[0] - bx) < m and abs(pos[1] - by) < m and pos[2] < TOP + 0.05


def yaw_de_agarre(robot, lado, yaw_bloque, puntos):
    """El bloque es simétrico cada 90°, así que hay varios giros de muñeca que sirven
    para cogerlo. Se prueba cada uno con la IK y se queda el que el brazo alcanza
    mejor en todos los puntos de la maniobra (algunos giros llevan la muñeca a su límite)."""
    base = (yaw_bloque + math.pi / 4) % (math.pi / 2) - math.pi / 4
    candidatos = {g: robot.error_ik(lado, puntos, g) for g in (base, base + math.pi / 2, base - math.pi / 2)}
    buenos = [g for g, e in candidatos.items() if e < 0.01]
    if buenos:      # entre los alcanzables, el que menos gira la muñeca
        return min(buenos, key=abs)
    return min(candidatos, key=candidatos.get)


# ==========================================================================
# Límites del espacio de trabajo
# ==========================================================================
def limitar(lado, pos, hombro, otro_y=None):
    x, y, z = pos
    y_min, y_max = BRAZOS[lado]["rango_y"]
    y = min(max(y, y_min), y_max)
    if otro_y is not None:          # no chocar con la otra pinza
        y = min(y, otro_y - SEPARACION_Y) if lado == "der" else max(y, otro_y + SEPARACION_Y)
    z = min(max(z, RANGO_Z[0]), RANGO_Z[1])
    # anillo de alcance alrededor del hombro (proyección radial)
    d = np.array([x - hombro[0], y - hombro[1]])
    r = float(np.linalg.norm(d))
    r_lim = min(max(r, ALCANCE[0]), ALCANCE[1])
    if r > 1e-6 and r_lim != r:
        x, y = hombro[0] + d[0] * r_lim / r, hombro[1] + d[1] * r_lim / r
    return np.array([x, y, z])


def pots_a_posicion(pots, lado):
    """Pot 1 -> X, Pot 2 -> Y, Pot 3 -> Z. Si al girar un pot la pinza se mueve al
    revés de lo que esperas, cambia su signo en INVERTIR (depende de a qué pata
    del potenciómetro conectaste 3V3 y GND)."""
    v = [4095 - a if inv else a for a, inv in zip(pots, INVERTIR)]
    return np.array([adc_a_rango(v[0], *RANGO_X),
                     adc_a_rango(v[1], *BRAZOS[lado]["rango_y"]),
                     adc_a_rango(v[2], *RANGO_Z)])


# ==========================================================================
# Rutina automática de pick-and-place
# ==========================================================================
def plan_rutina(robot, bloques, cliente):
    """Genera los pasos para llevar a la bandeja cada bloque que no esté ya en ella."""
    pasos = []
    bx, by = BANDEJA["centro"]
    for lado, nombre, dy in (("der", "rojo", -0.025), ("izq", "azul", 0.025)):
        pos, yaw = pose_bloque(bloques[nombre], cliente)
        if en_bandeja(pos):
            continue
        encima = (pos[0], pos[1], TOP + 0.15)
        agarre = (pos[0], pos[1], max(pos[2] - 0.005, TOP + 0.017))
        g = yaw_de_agarre(robot, lado, yaw, [encima, agarre, (bx, by + dy, TOP + 0.20), (bx, by + dy, TOP + 0.045)])
        pasos += [
            ("pinza", lado, True, 0.3, f"{lado}: abrir pinza"),
            ("ir", lado, encima, g, f"{lado}: sobre el bloque {nombre}"),
            ("ir", lado, agarre, g, f"{lado}: bajar al bloque {nombre}"),
            ("pinza", lado, False, 0.8, f"{lado}: cerrar pinza"),
            ("ir", lado, (pos[0], pos[1], TOP + 0.20), g, f"{lado}: levantar"),
            ("verificar", lado, nombre, 0, f"{lado}: verificar agarre del bloque {nombre}"),
            # se conserva el giro de muñeca del agarre: girarla con el bloque en la mano lo suelta
            ("ir", lado, (bx, by + dy, TOP + 0.20), g, f"{lado}: llevar a la bandeja"),
            ("ir", lado, (bx, by + dy, TOP + 0.045), g, f"{lado}: bajar sobre la bandeja"),
            ("pinza", lado, True, 0.6, f"{lado}: soltar"),
            ("ir", lado, (bx, by + dy, TOP + 0.20), g, f"{lado}: subir"),
            ("ir", lado, BRAZOS[lado]["inicio"], 0.0, f"{lado}: volver al inicio"),
        ]
    return pasos


# ==========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--sin-serial", action="store_true", help="usar la consola virtual (sliders)")
    ap.add_argument("--sin-ventana", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--duracion", type=float, default=0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    if not os.path.exists(URDF_BAXTER):
        print(f"No encuentro el modelo del Baxter en:\n  {URDF_BAXTER}")
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
    robot = Baxter(cliente)
    bloques = crear_escena(cliente)
    for lado, b in BRAZOS.items():
        robot.colocar(lado, b["inicio"])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=cliente)
    if not args.sin_ventana:
        # en modo virtual los paneles de PyBullet tapan los lados: cámara un poco más lejos
        p.resetDebugVisualizerCamera(cameraDistance=1.9 if args.sin_serial else 1.5, cameraYaw=120, cameraPitch=-30,
                                     cameraTargetPosition=[0.40, -0.28, -0.08], physicsClientId=cliente)

    # marcadores (esferas sin colisión) que muestran a dónde apunta cada consola
    marcador = {}
    for lado, b in BRAZOS.items():
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.012, rgbaColor=b["color"] + [0.55],
                                  physicsClientId=cliente)
        marcador[lado] = p.createMultiBody(0, -1, vis, b["inicio"], physicsClientId=cliente)

    if consola is not None:
        consola.start()
        print("Consola ESP32 conectada")
    else:
        consola = ConsolaVirtual(["X (cerca-lejos)", "Y (der-izq)", "Z (altura)"],
                                 ["Cambiar brazo", "Abrir/cerrar pinza", "Modo espejo",
                                  "Posicion inicial", "Rutina pick&place"],
                                 valores_iniciales=(1400, 900, 2600), cliente=cliente)
    hud = Hud(7, cliente, x_ndc=-0.48, y_ndc=0.84) if args.sin_serial else Hud(7, cliente)

    # ---- Registro ----
    f_log = open(os.path.join(carpeta_logs, f"baxter_{marca}.csv"), "w", newline="", encoding="utf-8")
    log = csv.writer(f_log)
    columnas = ["t_s", "modo", "brazo_activo", "paso_rutina", "evento"]
    for lado in BRAZOS:
        columnas += [f"{lado}_obj_{c}" for c in "xyz"] + [f"{lado}_suave_{c}" for c in "xyz"] \
                    + [f"{lado}_real_{c}" for c in "xyz"] + [f"{lado}_pinza_abierta", f"{lado}_abertura_m"]
    for nombre in BLOQUES_INICIO:
        columnas += [f"bloque_{nombre}_{c}" for c in "xyz"] + [f"bloque_{nombre}_agarrado"]
    log.writerow(columnas)

    # ---- Estado ----
    dt = 1.0 / CTRL_HZ
    sub = PYB_HZ // CTRL_HZ
    modo, activo = MANUAL, "der"
    objetivo = {l: np.array(b["inicio"], dtype=float) for l, b in BRAZOS.items()}
    yaw = {l: 0.0 for l in BRAZOS}
    suave = {l: Suavizador(robot.efector(l)[0]) for l in BRAZOS}
    pinza_abierta = {l: True for l in BRAZOS}
    siguiendo, pots_ref = False, None          # captura suave de los potenciómetros
    limitado_y = False                         # la otra pinza impide seguir moviendo en Y
    rutina, paso, t_paso, intentos = [], 0, 0.0, {}
    t_boton4 = None
    hombros = {l: robot.hombro(l) for l in BRAZOS}
    i, t_hud = 0, -1.0
    inicio_real = time.time()
    print("Listo. 1 = brazo | 2 = pinza | 3 = espejo | 4 = inicio (mantener: reiniciar bloques) | 5 = rutina")

    def evento_txt(txt):
        print(f"[{i * dt:7.2f}s] {txt}")
        return txt

    try:
        while True:
            t = i * dt
            if args.duracion and t > args.duracion:
                break
            evento = ""
            m = consola.leer()
            pots = m["pots"] if (m and consola.conectado()) else None

            # ---- 1) Pulsadores ----
            for btn in consola.presionados():
                if btn == 5:
                    if modo == RUTINA:
                        modo, rutina = MANUAL, []
                        evento = evento_txt("boton 5: rutina cancelada")
                    else:
                        rutina = plan_rutina(robot, bloques, cliente)
                        if rutina:
                            modo, paso, t_paso, intentos = RUTINA, 0, t, {}
                            evento = evento_txt(f"boton 5: rutina iniciada ({len(rutina)} pasos)")
                            evento_txt(rutina[0][4])
                        else:
                            evento = evento_txt("boton 5: los dos bloques ya están en la bandeja")
                    siguiendo, pots_ref = False, pots
                    continue
                if modo == RUTINA:
                    continue                       # durante la rutina solo el 5 hace algo
                if btn == 1:
                    activo = "izq" if activo == "der" else "der"
                    if modo == ESPEJO:
                        modo = MANUAL
                    siguiendo, pots_ref = False, pots
                    evento = evento_txt(f"boton 1: brazo activo {BRAZOS[activo]['nombre']}")
                elif btn == 2:
                    if modo == ESPEJO:
                        # si alguna está abierta se cierran las dos; si ambas están cerradas se abren
                        # (así nunca se suelta por sorpresa lo que tiene una de ellas)
                        lados, nuevo = list(BRAZOS), not any(pinza_abierta.values())
                    else:
                        lados, nuevo = [activo], not pinza_abierta[activo]
                    for l in lados:
                        pinza_abierta[l] = nuevo
                    evento = evento_txt(f"boton 2: pinza {'abierta' if nuevo else 'cerrada'}"
                                        + (" (ambas)" if modo == ESPEJO else f" ({BRAZOS[activo]['nombre'].lower()})"))
                elif btn == 3:
                    if modo == ESPEJO:
                        modo = MANUAL
                        evento = evento_txt("boton 3: modo espejo desactivado")
                    else:
                        modo, activo = ESPEJO, "der"
                        evento = evento_txt("boton 3: modo espejo (el izquierdo copia al derecho)")
                    siguiendo, pots_ref = False, pots
                elif btn == 4:
                    t_boton4 = t
                    for l in BRAZOS:
                        objetivo[l] = np.array(BRAZOS[l]["inicio"], dtype=float)
                        yaw[l] = 0.0
                    if modo == ESPEJO:
                        modo = MANUAL
                    siguiendo, pots_ref = False, pots
                    evento = evento_txt("boton 4: posición inicial")

            # pulsador 4 mantenido 1.5 s (o tecla R) -> reiniciar los bloques
            teclas = p.getKeyboardEvents(physicsClientId=cliente) if not args.sin_ventana else {}
            mantenido = t_boton4 is not None and consola.mantenido(4) and t - t_boton4 >= 1.5
            if mantenido or (ord("r") in teclas and teclas[ord("r")] & p.KEY_WAS_TRIGGERED):
                t_boton4 = None
                robot.soltar_todo()
                reiniciar_bloques(bloques, cliente)
                if modo == RUTINA:
                    modo, rutina = MANUAL, []
                evento = evento_txt("bloques reiniciados")
            if t_boton4 is not None and not consola.mantenido(4) and t - t_boton4 > 0.1:
                t_boton4 = None

            # ---- 2) Objetivos: consola (manual/espejo) o rutina ----
            if modo in (MANUAL, ESPEJO) and pots is not None:
                if not siguiendo:
                    if pots_ref is None:
                        pots_ref = pots
                    elif max(abs(a - b) for a, b in zip(pots, pots_ref)) > UMBRAL_CAPTURA:
                        siguiendo = True
                limitado_y = False
                if siguiendo:
                    otro = "izq" if activo == "der" else "der"
                    if modo == ESPEJO:
                        pos = pots_a_posicion(pots, "der")
                        pos[1] = min(pos[1], -SEPARACION_Y / 2)
                        objetivo["der"] = limitar("der", pos, hombros["der"])
                        espejo = objetivo["der"].copy()
                        espejo[1] = -espejo[1]
                        objetivo["izq"] = limitar("izq", espejo, hombros["izq"])
                    else:
                        deseado = limitar(activo, pots_a_posicion(pots, activo), hombros[activo])
                        objetivo[activo] = limitar(activo, deseado, hombros[activo], otro_y=suave[otro].pos[1])
                        limitado_y = abs(objetivo[activo][1] - deseado[1]) > 0.002
                    yaw[activo] = 0.0

            elif modo == RUTINA and rutina:
                tipo, lado, dato, extra, texto = rutina[paso]
                terminado = False
                if tipo == "ir":
                    objetivo[lado] = np.array(dato, dtype=float)
                    yaw[lado] = extra
                    real = robot.efector(lado)[0]
                    terminado = (suave[lado].llego(objetivo[lado]) and np.linalg.norm(real - objetivo[lado]) < 0.012) \
                        or t - t_paso > 6.0
                elif tipo == "pinza":
                    pinza_abierta[lado] = dato
                    terminado = t - t_paso >= extra
                elif tipo == "verificar":
                    sujeto = robot.sujecion[lado]
                    if (sujeto and sujeto[1] == dato) or robot.sostiene(lado, bloques[dato]):
                        terminado = True
                        evento = evento_txt(f"agarre confirmado: bloque {dato}")
                    else:
                        intentos[dato] = intentos.get(dato, 0) + 1
                        evento = evento_txt(f"agarre fallido del bloque {dato} (intento {intentos[dato]})")
                        pinza_abierta[lado] = True
                        if intentos[dato] < 3:          # replanificar con la posición actual del bloque
                            rutina = plan_rutina(robot, bloques, cliente)
                            paso, t_paso = 0, t
                        else:
                            terminado = True
                if terminado:
                    paso, t_paso = paso + 1, t
                    if paso >= len(rutina):
                        n = sum(en_bandeja(pose_bloque(c, cliente)[0]) for c in bloques.values())
                        evento = evento_txt(f"rutina completada: {n}/2 bloques en la bandeja")
                        modo, rutina = MANUAL, []
                        siguiendo, pots_ref = False, pots
                    else:
                        evento_txt(rutina[paso][4])

            # ---- 3) Trayectoria suave + cinemática inversa ----
            vmax = VMAX_RUTINA if modo == RUTINA else VMAX_MANUAL
            for lado in BRAZOS:
                robot.ik(lado, suave[lado].paso(objetivo[lado], dt, vmax, AMAX), yaw[lado])
            robot.aplicar(pinza_abierta)
            for _ in range(sub):
                p.stepSimulation(physicsClientId=cliente)
            cambio = robot.actualizar_sujecion(pinza_abierta, bloques, dt)
            if cambio:
                evento = (evento + "; " if evento else "") + evento_txt(cambio)

            # ---- 4) Registro ----
            reales = {l: robot.efector(l)[0] for l in BRAZOS}
            fila = [f"{t:.3f}", modo, activo, f"{paso + 1}/{len(rutina)}" if modo == RUTINA else "", evento]
            for lado in BRAZOS:
                fila += [f"{v:.4f}" for v in objetivo[lado]] + [f"{v:.4f}" for v in suave[lado].pos] \
                        + [f"{v:.4f}" for v in reales[lado]] + [int(pinza_abierta[lado]), f"{robot.abertura(lado):.4f}"]
            agarrados = {}
            for nombre, c in bloques.items():
                agarrados[nombre] = next((l for l in BRAZOS if (robot.sujecion[l] and robot.sujecion[l][1] == nombre)
                                          or robot.sostiene(l, c)), "")
                fila += [f"{v:.4f}" for v in pose_bloque(c, cliente)[0]] + [agarrados[nombre]]
            log.writerow(fila)

            # ---- 5) Indicador en pantalla ----
            if not args.sin_ventana:
                for lado in BRAZOS:
                    p.resetBasePositionAndOrientation(marcador[lado], objetivo[lado].tolist(), [0, 0, 0, 1],
                                                      physicsClientId=cliente)
                if t - t_hud >= 0.2:
                    t_hud = t
                    err = {l: np.linalg.norm(reales[l] - suave[l].pos) * 100 for l in BRAZOS}
                    if modo == RUTINA:
                        linea1 = f"RUTINA pick&place  paso {paso + 1}/{len(rutina)}: {rutina[paso][4]}"
                    elif modo == ESPEJO:
                        linea1 = "Modo ESPEJO: los dos brazos a la vez"
                    else:
                        linea1 = f"Brazo activo: {BRAZOS[activo]['nombre']}"
                    if modo != RUTINA and not siguiendo:
                        linea1 += "   (mueve un pot para tomar el control)"
                    elif modo == MANUAL and limitado_y:
                        linea1 += "   (Y limitado: la otra pinza está cerca)"

                    def txt_pinza(l):
                        bloque = next((n for n, a in agarrados.items() if a == l), None)
                        return (f"{'ABIERTA' if pinza_abierta[l] else 'CERRADA'}"
                                + (f" con bloque {bloque}" if bloque else ""))
                    n_bandeja = sum(en_bandeja(pose_bloque(c, cliente)[0]) for c in bloques.values())
                    o = objetivo[activo]
                    hud.actualizar([
                        (linea1, BRAZOS[activo]["color"] if modo != RUTINA else [0.55, 0.3, 0.0]),
                        (f"Objetivo {BRAZOS[activo]['nombre'].lower()}: x {o[0]:.2f}  y {o[1]:+.2f}  "
                         f"z {o[2] - TOP:.2f} m sobre la mesa", [0.2, 0.2, 0.2]),
                        (f"Seguimiento: der {err['der']:.1f} cm | izq {err['izq']:.1f} cm", [0.2, 0.2, 0.2]),
                        (f"Pinza der: {txt_pinza('der')} | Pinza izq: {txt_pinza('izq')}", [0.2, 0.2, 0.2]),
                        (f"Bloques en la bandeja: {n_bandeja}/2", [0.1, 0.5, 0.2]),
                        (consola.texto_estado(), [0, 0.45, 0] if consola.conectado() else [0.8, 0, 0]),
                        ("1=Brazo 2=Pinza 3=Espejo 4=Inicio (mantener: bloques) 5=Rutina", [0.35, 0.35, 0.35]),
                    ])

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
