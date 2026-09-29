"""
Taller Segundo Corte - Inciso (a): drones A -> B -> C controlados desde la ESP32
Micros y Laboratorio - UMNG

Una formación de 6 cuadricópteros Crazyflie (gym-pybullet-drones, CtrlAviary +
DSLPIDControl) vuela entre tres puntos A, B y C. Todo se manda desde la
consola ESP32:

    Pulsador 1 -> ir a A          Pot 1 -> altura de vuelo      (0.3 .. 1.5 m)
    Pulsador 2 -> ir a B          Pot 2 -> velocidad de crucero (0.1 .. 1.0 m/s)
    Pulsador 3 -> ir a C          Pot 3 -> radio de la formación (0.25 .. 0.7 m)
    Pulsador 4 -> despegar / aterrizar
    Pulsador 5 -> ruta automática A -> B -> C (otra pulsación la cancela)

Uso:
    python drones_esp32.py                 # consola ESP32 en COM5
    python drones_esp32.py --port COM7
    python drones_esp32.py --sin-serial    # consola virtual (sliders)
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
sys.path.insert(0, os.path.dirname(AQUI))
from comun.consola import ConsolaESP32, ConsolaVirtual, Hud, adc_a_rango  # noqa: E402

from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl  # noqa: E402
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary  # noqa: E402
from gym_pybullet_drones.utils.enums import DroneModel, Physics  # noqa: E402

# ---------------- Parámetros de la misión ----------------
N_DRONES = 6
PUNTOS = {"A": np.array([0.0, 0.0]), "B": np.array([1.2, 1.0]), "C": np.array([1.2, -1.2])}
COLORES = {"A": [0.85, 0.15, 0.15], "B": [0.15, 0.45, 0.9], "C": [0.1, 0.65, 0.2]}
RANGO_ALTURA = (0.3, 1.5)       # m    (pot 1)
RANGO_VEL = (0.1, 1.0)          # m/s  (pot 2)
RANGO_RADIO = (0.25, 0.7)       # m    (pot 3)
VEL_VERTICAL = 0.4              # m/s  despegue / aterrizaje / cambio de altura
VEL_RADIO = 0.2                 # m/s  cambio de radio de la formación
ACEL_MAX = 0.4                  # m/s² aceleración/frenado del perfil trapezoidal
Z_INICIAL = 0.05
Z_ATERRIZAJE = 0.05
TOLERANCIA_LLEGADA = 0.03       # m    error máximo del centro para "llegó"
ESPERA_RUTA = 2.0               # s    tiempo en cada punto durante la ruta
PYB_HZ, CTRL_HZ = 240, 48

EN_TIERRA, DESPEGANDO, EN_VUELO, ATERRIZANDO = "EN TIERRA", "DESPEGANDO", "EN VUELO", "ATERRIZANDO"


def acercar(actual, objetivo, paso):
    """Mueve 'actual' hacia 'objetivo' como máximo 'paso' (limitador de velocidad)."""
    diferencia = objetivo - actual
    distancia = float(np.linalg.norm(diferencia))
    if distancia <= paso or distancia == 0:
        return np.array(objetivo, dtype=float), np.zeros_like(diferencia)
    direccion = diferencia / distancia
    return actual + direccion * paso, direccion


def offsets_formacion(radio):
    return np.array([[radio * math.cos(2 * math.pi * i / N_DRONES),
                      radio * math.sin(2 * math.pi * i / N_DRONES)] for i in range(N_DRONES)])


def dibujar_puntos(cliente):
    for nombre, xy in PUNTOS.items():
        color = COLORES[nombre]
        disco = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.004,
                                    rgbaColor=color + [0.9], physicsClientId=cliente)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=disco,
                          basePosition=[xy[0], xy[1], 0.003], physicsClientId=cliente)
        p.addUserDebugText(nombre, [xy[0], xy[1], 0.25], color, textSize=2.5,
                           physicsClientId=cliente)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--sin-serial", action="store_true", help="usar la consola virtual (sliders)")
    ap.add_argument("--sin-ventana", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--duracion", type=float, default=0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    marca = time.strftime("%Y%m%d_%H%M%S")
    carpeta_logs = os.path.join(AQUI, "logs")
    os.makedirs(carpeta_logs, exist_ok=True)

    # ---- Consola real (se abre antes que la simulación para detectar errores de puerto) ----
    consola = None
    if not args.sin_serial:
        try:
            consola = ConsolaESP32(args.port, archivo_tramas=os.path.join(carpeta_logs, f"tramas_{marca}.csv"))
        except Exception as e:
            print(f"No se pudo abrir {args.port}: {e}")
            print("Cierra el Serial Monitor de PlatformIO y revisa el puerto.")
            return
        print(f"Puerto {args.port} abierto")

    # ---- Entorno de gym-pybullet-drones ----
    radio = 0.4
    inicio = np.hstack([PUNTOS["A"] + offsets_formacion(radio), np.full((N_DRONES, 1), Z_INICIAL)])
    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=N_DRONES, initial_xyzs=inicio,
                     initial_rpys=np.zeros((N_DRONES, 3)), physics=Physics.PYB,
                     pyb_freq=PYB_HZ, ctrl_freq=CTRL_HZ, gui=not args.sin_ventana,
                     obstacles=True, user_debug_gui=False)
    cliente = env.getPyBulletClient()
    ctrl = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(N_DRONES)]
    dt = env.CTRL_TIMESTEP

    if not args.sin_ventana:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=cliente)
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=-35, cameraPitch=-40,
                                     cameraTargetPosition=[0.7, -0.1, 0.3], physicsClientId=cliente)
    dibujar_puntos(cliente)

    if consola is not None:
        consola.start()                 # el hilo serial arranca con la simulación ya cargada
        print("Consola ESP32 conectada")
    else:
        consola = ConsolaVirtual(["Altura", "Velocidad", "Radio formacion"],
                                 ["Ir a A", "Ir a B", "Ir a C", "Despegar/Aterrizar", "Ruta A-B-C"],
                                 valores_iniciales=(1700, 1800, 1500), cliente=cliente)
    # en modo virtual PyBullet muestra paneles a los lados y un menú arriba
    hud = Hud(6, cliente, x_ndc=-0.48, y_ndc=0.84) if args.sin_serial else Hud(6, cliente)

    # ---- Registro del vuelo ----
    f_log = open(os.path.join(carpeta_logs, f"vuelo_{marca}.csv"), "w", newline="", encoding="utf-8")
    log = csv.writer(f_log)
    log.writerow(["t_s", "estado", "destino", "ruta_paso", "altura_obj", "vel_obj", "radio_obj",
                  "ref_x", "ref_y", "ref_z", "centro_x", "centro_y", "centro_z", "error_m", "evento",
                  *[f"d{j}_{c}" for j in range(N_DRONES) for c in "xyz"]])

    # ---- Estado de la misión ----
    estado = EN_TIERRA
    destino = None
    ruta, paso_ruta, t_llegada = None, 0, None
    altura_obj, vel_obj, radio_obj = 0.9, 0.5, radio
    ref_xy = PUNTOS["A"].astype(float).copy()
    ref_z = Z_INICIAL
    ref_r = radio
    v_actual = 0.0                  # velocidad del perfil trapezoidal
    accion = np.zeros((N_DRONES, 4))
    obs, _, _, _, _ = env.step(accion)
    i, t_hud, t_traza = 0, -1.0, 0.0
    pos_prev = None
    inicio_real = time.time()
    print("Listo. Pulsador 4 = despegar | 1/2/3 = ir a A/B/C | 5 = ruta A->B->C")

    try:
        while True:
            t = i * dt
            if args.duracion and t > args.duracion:
                break
            posiciones = obs[:, 0:3]
            centro = posiciones.mean(axis=0)
            evento = ""

            # ---- 1) Leer la consola ----
            m = consola.leer()
            if m and consola.conectado():
                altura_obj = adc_a_rango(m["pots"][0], *RANGO_ALTURA)
                vel_obj = adc_a_rango(m["pots"][1], *RANGO_VEL)
                radio_obj = adc_a_rango(m["pots"][2], *RANGO_RADIO)

            for b in consola.presionados():
                if b in (1, 2, 3):
                    destino, ruta, v_actual = "ABC"[b - 1], None, 0.0
                    evento = f"boton {b}: ir a {destino}"
                    if estado in (EN_TIERRA, ATERRIZANDO):
                        estado = DESPEGANDO
                elif b == 4:
                    ruta = None
                    if estado in (EN_TIERRA, ATERRIZANDO):
                        estado, evento = DESPEGANDO, "boton 4: despegar"
                    else:
                        estado, evento = ATERRIZANDO, "boton 4: aterrizar"
                elif b == 5:
                    if ruta:
                        ruta, evento = None, "boton 5: ruta cancelada"
                    else:
                        ruta, paso_ruta, t_llegada = ["A", "B", "C"], 0, None
                        destino, evento = "A", "boton 5: ruta A-B-C"
                        if estado in (EN_TIERRA, ATERRIZANDO):
                            estado = DESPEGANDO
                if evento:
                    print(f"[{t:6.2f}s] {evento}")

            # ---- 2) Máquina de estados y referencias suaves ----
            vel_ff = np.zeros(3)
            if estado == EN_TIERRA:
                ref_xy, ref_z = centro[:2].copy(), centro[2]
            elif estado == DESPEGANDO:
                ref_z = float(acercar(np.array([ref_z]), np.array([altura_obj]), VEL_VERTICAL * dt)[0][0])
                if abs(centro[2] - altura_obj) < 0.05:
                    estado, evento = EN_VUELO, "despegue completo"
            elif estado == EN_VUELO:
                ref_z = float(acercar(np.array([ref_z]), np.array([altura_obj]), VEL_VERTICAL * dt)[0][0])
                if destino:
                    # Perfil trapezoidal: acelera, crucero a vel_obj y frena antes de llegar
                    restante = float(np.linalg.norm(PUNTOS[destino] - ref_xy))
                    v_frenado = math.sqrt(2 * ACEL_MAX * restante)
                    v_actual = min(v_actual + ACEL_MAX * dt, vel_obj, v_frenado)
                    ref_xy, direccion = acercar(ref_xy, PUNTOS[destino], v_actual * dt)
                    vel_ff[:2] = direccion * v_actual
                    if restante == 0:
                        v_actual = 0.0
            elif estado == ATERRIZANDO:
                ref_z = float(acercar(np.array([ref_z]), np.array([Z_ATERRIZAJE]), VEL_VERTICAL * dt)[0][0])
                if ref_z <= Z_ATERRIZAJE + 1e-6 and posiciones[:, 2].max() < 0.08:
                    estado, destino, evento = EN_TIERRA, None, "aterrizaje completo"
            ref_r = float(acercar(np.array([ref_r]), np.array([radio_obj]), VEL_RADIO * dt)[0][0])

            # ---- 3) Llegada a un punto y avance de la ruta ----
            error = float(np.linalg.norm(centro[:2] - PUNTOS[destino])) if destino else 0.0
            if estado == EN_VUELO and destino and np.allclose(ref_xy, PUNTOS[destino]) \
                    and error < TOLERANCIA_LLEGADA:
                if t_llegada is None:
                    t_llegada = t
                    evento = f"llegada a {destino} (error {error * 100:.1f} cm)"
                if ruta and t - t_llegada >= ESPERA_RUTA:
                    if paso_ruta < len(ruta) - 1:
                        paso_ruta += 1
                        destino, t_llegada = ruta[paso_ruta], None
                        evento = f"ruta: siguiente punto {destino}"
                    else:
                        ruta, evento = None, "ruta A-B-C completada"
            elif destino and not np.allclose(ref_xy, PUNTOS[destino]):
                t_llegada = None
            if evento and not evento.startswith("boton"):
                print(f"[{t:6.2f}s] {evento}")

            # ---- 4) Control PID de cada dron ----
            if estado == EN_TIERRA:
                accion[:] = 0.0
            else:
                objetivos = np.hstack([ref_xy + offsets_formacion(ref_r), np.full((N_DRONES, 1), ref_z)])
                for j in range(N_DRONES):
                    accion[j, :], _, _ = ctrl[j].computeControlFromState(
                        control_timestep=dt, state=obs[j], target_pos=objetivos[j],
                        target_rpy=np.zeros(3), target_vel=vel_ff)
            obs, _, _, _, _ = env.step(accion)

            # ---- 5) Registro ----
            log.writerow([f"{t:.3f}", estado, destino or "", f"{paso_ruta + 1}/3" if ruta else "",
                          f"{altura_obj:.3f}", f"{vel_obj:.3f}", f"{radio_obj:.3f}",
                          f"{ref_xy[0]:.4f}", f"{ref_xy[1]:.4f}", f"{ref_z:.4f}",
                          f"{centro[0]:.4f}", f"{centro[1]:.4f}", f"{centro[2]:.4f}",
                          f"{error:.4f}", evento, *[f"{v:.4f}" for v in posiciones.flatten()]])

            # ---- 6) Visualización: estelas e indicador ----
            if not args.sin_ventana:
                if t - t_traza >= 0.1:
                    t_traza = t
                    if pos_prev is not None and estado != EN_TIERRA:
                        for j in range(N_DRONES):
                            p.addUserDebugLine(pos_prev[j], posiciones[j], [0.1, 0.7, 0.3], 1.5,
                                               lifeTime=8, physicsClientId=cliente)
                    pos_prev = posiciones.copy()
                if t - t_hud >= 0.25:
                    t_hud = t
                    color = COLORES.get(destino, [0.1, 0.1, 0.1])
                    hud.actualizar([
                        (f"Estado: {estado}" + (f"  ->  destino {destino}" if destino else ""), color),
                        (f"Ruta A-B-C: paso {paso_ruta + 1}/3" if ruta else "Ruta: inactiva", [0.2, 0.2, 0.2]),
                        (f"Altura {altura_obj:.2f} m | Vel {vel_obj:.2f} m/s | Radio {radio_obj:.2f} m", [0.2, 0.2, 0.2]),
                        (f"Centro ({centro[0]:+.2f}, {centro[1]:+.2f}, {centro[2]:.2f}) | error {error * 100:.1f} cm", [0.2, 0.2, 0.2]),
                        (consola.texto_estado(), [0, 0.45, 0] if consola.conectado() else [0.8, 0, 0]),
                        ("1=A  2=B  3=C  4=Despegar/Aterrizar  5=Ruta A-B-C", [0.35, 0.35, 0.35]),
                    ])
            # ---- 7) Mantener la simulación en tiempo real ----
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
            env.close()
        except p.error:
            pass
        print(f"Registros guardados en: {carpeta_logs}")


if __name__ == "__main__":
    main()
