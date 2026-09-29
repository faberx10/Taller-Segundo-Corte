"""
Taller Segundo Corte - Módulo común de la consola ESP32
Micros y Laboratorio - UMNG

Lo usan los tres incisos del taller. Ofrece la misma interfaz para:
  - ConsolaESP32:   la consola real (ESP32 por USB-Serial)
  - ConsolaVirtual: sliders y botones dentro de PyBullet (probar sin hardware)
y un indicador de texto (Hud) que sigue a la cámara de PyBullet.

Trama que envía el ESP32 a 50 Hz:
    $<seq>,<t_ms>,<pot1>,<pot2>,<pot3>,<botones>,<c1>,<c2>,<c3>,<c4>,<c5>*<CS>
"""

import csv
import os
import re
import statistics
import threading
import time
from collections import deque

import numpy as np
import pybullet as p
import serial

TRAMA = re.compile(r"^\$([^*]+)\*([0-9A-Fa-f]{2})$")
N_CAMPOS = 11
ADC_MIN, ADC_MAX = 60, 4035       # zonas muertas del ADC del ESP32


def adc_a_rango(adc, bajo, alto):
    """Convierte una lectura de 12 bits (0..4095) al rango [bajo, alto]."""
    n = (adc - ADC_MIN) / (ADC_MAX - ADC_MIN)
    n = min(max(n, 0.0), 1.0)
    return bajo + n * (alto - bajo)


class _BaseConsola:
    """Lógica compartida: detección de pulsaciones por contador."""

    def __init__(self):
        self._contadores_previos = None

    def presionados(self):
        """Lista de pulsadores (1..5) presionados desde la última llamada."""
        m = self.leer()
        if m is None:
            return []
        c = m["contadores"]
        if self._contadores_previos is None:
            self._contadores_previos = list(c)
            return []
        eventos = [i + 1 for i in range(5) if c[i] != self._contadores_previos[i]]
        self._contadores_previos = list(c)
        return eventos

    def mantenido(self, n):
        """True si el pulsador n (1..5) está presionado en este instante."""
        m = self.leer()
        return bool(m and (m["botones"] >> (n - 1)) & 1)


# --------------------------------------------------------------------------
# Consola real
# --------------------------------------------------------------------------
class ConsolaESP32(_BaseConsola, threading.Thread):
    def __init__(self, puerto="COM5", baudios=115200, archivo_tramas=None):
        _BaseConsola.__init__(self)
        threading.Thread.__init__(self, daemon=True)
        self.ser = serial.Serial()
        self.ser.port = puerto
        self.ser.baudrate = baudios
        self.ser.timeout = 0.02
        self.ser.dtr = False          # no reiniciar el ESP32 al abrir el puerto
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.lock = threading.Lock()
        self.corriendo = True
        self._muestra = None
        self._t0 = time.perf_counter()
        self.tramas_ok = self.tramas_malas = self.tramas_perdidas = 0
        self._seq_anterior = None
        self.llegadas = deque(maxlen=100)
        self.rtts = deque(maxlen=60)
        self._rtt_nuevo = None
        self._pings = {}
        self._n_ping = 0

        self._f_log = None
        if archivo_tramas:
            os.makedirs(os.path.dirname(archivo_tramas), exist_ok=True)
            self._f_log = open(archivo_tramas, "w", newline="", encoding="utf-8")
            self._log = csv.writer(self._f_log)
            self._log.writerow(["t_pc_s", "seq", "t_esp_ms", "pot1", "pot2", "pot3", "botones",
                                "c1", "c2", "c3", "c4", "c5", "tramas_ok", "tramas_malas",
                                "tramas_perdidas", "rtt_ms"])

    # ---- hilo de lectura ----
    def run(self):
        self.ser.reset_input_buffer()    # descarta lo acumulado antes de arrancar
        self._t0 = time.perf_counter()
        buf = b""
        t_ping = 0.0
        while self.corriendo:
            ahora = time.perf_counter()
            if ahora - t_ping >= 1.0:
                self._n_ping += 1
                self._pings[self._n_ping] = time.perf_counter()
                try:
                    self.ser.write(f"P{self._n_ping}\n".encode())
                except serial.SerialException:
                    break
                t_ping = ahora
            try:
                datos = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                print("Se perdió la conexión con el ESP32.")
                break
            if not datos:
                continue
            buf += datos
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                # Marca de tiempo POR LÍNEA: si llegó una ráfaga de varias
                # tramas ya bufferizadas, cada una debe quedar con su propio
                # instante, no todas con el mismo (eso rompía las cuentas de Hz).
                self._procesar(linea.decode(errors="ignore").strip(), time.perf_counter())

    def _procesar(self, linea, t):
        if not linea:
            return
        if linea.startswith("#PONG,"):
            try:
                t_envio = self._pings.pop(int(linea[6:]), None)
            except ValueError:
                return
            if t_envio is not None:
                with self.lock:
                    rtt = (t - t_envio) * 1000.0
                    self.rtts.append(rtt)
                    self._rtt_nuevo = rtt
            return
        if linea.startswith("#"):
            print("ESP32:", linea[1:])
            return

        m = TRAMA.match(linea)
        campos = []
        if m:
            cs = 0
            for c in m.group(1):
                cs ^= ord(c)
            try:
                campos = [int(x) for x in m.group(1).split(",")]
            except ValueError:
                campos = []
            if cs != int(m.group(2), 16):
                campos = []
        if len(campos) != N_CAMPOS:
            with self.lock:
                self.tramas_malas += 1
            return

        seq = campos[0]
        with self.lock:
            if self._seq_anterior is not None:
                salto = (seq - self._seq_anterior - 1) % 65536
                if salto < 1000:                       # >1000 = el ESP32 se reinició
                    self.tramas_perdidas += salto
            self._seq_anterior = seq
            self.tramas_ok += 1
            self.llegadas.append(t)
            self._muestra = {"seq": seq, "t_esp": campos[1], "pots": campos[2:5],
                             "botones": campos[5], "contadores": campos[6:11], "t_llegada": t}
            if self._f_log:
                rtt, self._rtt_nuevo = self._rtt_nuevo, None
                self._log.writerow([f"{t - self._t0:.4f}", *campos,
                                    self.tramas_ok, self.tramas_malas, self.tramas_perdidas,
                                    f"{rtt:.2f}" if rtt is not None else ""])

    # ---- interfaz pública ----
    def leer(self):
        with self.lock:
            return dict(self._muestra) if self._muestra else None

    def conectado(self, timeout=0.5):
        with self.lock:
            return self._muestra is not None and time.perf_counter() - self._muestra["t_llegada"] < timeout

    def estadisticas(self):
        with self.lock:
            lleg, rtts = list(self.llegadas), list(self.rtts)
            est = {"ok": self.tramas_ok, "malas": self.tramas_malas, "perdidas": self.tramas_perdidas}
        duracion = lleg[-1] - lleg[0] if len(lleg) > 2 else 0.0
        if len(lleg) > 2 and duracion > 0:
            intervalos = [(b - a) * 1000 for a, b in zip(lleg, lleg[1:])]
            est["hz"] = (len(lleg) - 1) / duracion
            est["jitter"] = statistics.pstdev(intervalos)
        else:
            # Recién conectado, o llegó una ráfaga de tramas ya bufferizadas
            # con marcas de tiempo casi iguales: todavía no hay con qué
            # calcular una frecuencia real, así que se reporta 0 en vez de
            # dividir por (casi) cero.
            est["hz"] = est["jitter"] = 0.0
        est["rtt"] = statistics.median(rtts) if rtts else float("nan")
        return est

    def texto_estado(self):
        e = self.estadisticas()
        return (f"ESP32 {'OK' if self.conectado() else 'SIN DATOS'} | {e['hz']:.0f} Hz | "
                f"RTT {e['rtt']:.1f} ms | perdidas {e['perdidas']} | malas {e['malas']}")

    def cerrar(self):
        self.corriendo = False
        time.sleep(0.05)
        try:
            self.ser.close()
        finally:
            if self._f_log:
                self._f_log.close()


# --------------------------------------------------------------------------
# Consola virtual (sliders y botones en la ventana de PyBullet)
# --------------------------------------------------------------------------
class ConsolaVirtual(_BaseConsola):
    def __init__(self, nombres_pots, nombres_botones, valores_iniciales=(2048, 2048, 2048),
                 cliente=0):
        super().__init__()
        self.cliente = cliente
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=cliente)
        self._pots = [p.addUserDebugParameter(f"Pot {i + 1}: {n}", 0, 4095, v, physicsClientId=cliente)
                      for i, (n, v) in enumerate(zip(nombres_pots, valores_iniciales))]
        self._btns = [p.addUserDebugParameter(f"Boton {i + 1}: {n}", 1, 0, 0, physicsClientId=cliente)
                      for i, n in enumerate(nombres_botones)]
        self._t0 = time.perf_counter()

    def leer(self):
        try:
            pots = [int(p.readUserDebugParameter(s, physicsClientId=self.cliente)) for s in self._pots]
            cont = [int(p.readUserDebugParameter(b, physicsClientId=self.cliente)) for b in self._btns]
        except p.error:
            return None
        return {"seq": 0, "t_esp": 0, "pots": pots, "botones": 0, "contadores": cont,
                "t_llegada": time.perf_counter()}

    def conectado(self, timeout=0.5):
        return True

    def estadisticas(self):
        return {"ok": 0, "malas": 0, "perdidas": 0, "hz": 0.0, "jitter": 0.0, "rtt": float("nan")}

    def texto_estado(self):
        return "Consola virtual (sliders)"

    def cerrar(self):
        pass


# --------------------------------------------------------------------------
# Indicador de texto que sigue a la cámara
# --------------------------------------------------------------------------
class Hud:
    """Escribe varias líneas de texto fijas en la esquina superior izquierda de la vista.

    Usa las matrices de vista y proyección de la cámara de PyBullet para
    "desproyectar" un punto de la pantalla al mundo 3D, así el texto queda en la
    misma esquina aunque se gire la cámara o cambie el tamaño de la ventana.
    """

    def __init__(self, n_lineas, cliente=0, tamano=1.2, x_ndc=-0.97, y_ndc=0.92, paso_ndc=0.07, cerca=1.0):
        """cerca < 1 dibuja el texto más cerca de la cámara (fracción de la distancia al
        punto que mira), para que los objetos de una escena grande no lo tapen."""
        self.ids = [-1] * n_lineas
        self.cliente = cliente
        self.tamano = tamano
        self.x_ndc, self.y_ndc, self.paso_ndc = x_ndc, y_ndc, paso_ndc
        self.cerca = cerca

    def _a_mundo(self, inv, x, y, z):
        v = inv @ np.array([x, y, z, 1.0])
        return (v[:3] / v[3]).tolist()

    def actualizar(self, lineas):
        try:
            cam = p.getDebugVisualizerCamera(physicsClientId=self.cliente)
        except p.error:
            return
        if not cam or cam[0] == 0:          # sin ventana (modo DIRECT)
            return
        vista = np.array(cam[2]).reshape(4, 4).T          # PyBullet guarda por columnas
        proy = np.array(cam[3]).reshape(4, 4).T
        m = proy @ vista
        adelante = np.array(cam[5]) / np.linalg.norm(cam[5])
        objetivo = np.array([*(np.array(cam[11]) - adelante * cam[10] * (1.0 - self.cerca)), 1.0])
        clip = m @ objetivo
        z_ndc = clip[2] / clip[3]                          # profundidad del punto que mira la cámara
        inv = np.linalg.inv(m)
        for k, (texto, color) in enumerate(lineas[:len(self.ids)]):
            pos = self._a_mundo(inv, self.x_ndc, self.y_ndc - self.paso_ndc * k, z_ndc)
            self.ids[k] = p.addUserDebugText(texto, pos, color, textSize=self.tamano,
                                             replaceItemUniqueId=self.ids[k],
                                             physicsClientId=self.cliente)
