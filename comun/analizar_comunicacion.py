"""
Taller Segundo Corte - Validación de la comunicación en tiempo real
Sirve para los tres incisos. Lee un registro tramas_*.csv (lo escribe la clase
ConsolaESP32 con UNA fila por cada trama recibida) y calcula:
  - frecuencia de recepción y periodo de muestreo del ESP32
  - intervalo de llegada al PC y jitter
  - tramas perdidas y con checksum inválido
  - RTT (ping-pong PC -> ESP32 -> PC)
Guarda la figura tramas_*_comunicacion.png junto al registro.

Uso:
    python analizar_comunicacion.py ../a_drones/logs/tramas_XXXX.csv
    python analizar_comunicacion.py ../a_drones/logs      (toma el más reciente)
"""

import csv
import glob
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def percentil(datos, q):
    datos = sorted(datos)
    k = (len(datos) - 1) * q / 100
    i = int(k)
    return datos[i] if i + 1 >= len(datos) else datos[i] + (datos[i + 1] - datos[i]) * (k - i)


def analizar(ruta):
    with open(ruta, newline="") as f:
        filas = list(csv.DictReader(f))
    if len(filas) < 10:
        print("El registro tiene muy pocas tramas.")
        return None

    t = [float(r["t_pc_s"]) for r in filas]
    seq = [int(r["seq"]) for r in filas]
    t_esp = [float(r["t_esp_ms"]) for r in filas]
    d_seq = [(b - a) % 65536 for a, b in zip(seq, seq[1:])]
    llegada = [(t[i + 1] - t[i]) * 1000 for i, d in enumerate(d_seq) if d == 1]
    periodo_esp = [(t_esp[i + 1] - t_esp[i]) / d for i, d in enumerate(d_seq) if d >= 1]
    rtt = [float(r["rtt_ms"]) for r in filas if r["rtt_ms"]]
    t_rtt = [float(r["t_pc_s"]) for r in filas if r["rtt_ms"]]
    rtt_estable, t_rtt_estable = rtt[1:], t_rtt[1:]     # la 1.a se toma al abrir el puerto
    ultima = filas[-1]
    ok, malas, perdidas = int(ultima["tramas_ok"]), int(ultima["tramas_malas"]), int(ultima["tramas_perdidas"])
    duracion = t[-1] - t[0]

    r = {
        "duracion_s": duracion, "tramas_ok": ok, "tramas_malas": malas, "tramas_perdidas": perdidas,
        "frecuencia_hz": (len(filas) - 1) / duracion,
        "periodo_esp_ms": statistics.mean(periodo_esp),
        "periodo_esp_min": min(periodo_esp), "periodo_esp_max": max(periodo_esp),
        "llegada_media_ms": statistics.mean(llegada), "jitter_ms": statistics.pstdev(llegada),
        "llegada_p95_ms": percentil(llegada, 95), "llegada_max_ms": max(llegada),
    }
    if rtt_estable:
        r.update({"rtt_mediana_ms": statistics.median(rtt_estable), "rtt_media_ms": statistics.mean(rtt_estable),
                  "rtt_min_ms": min(rtt_estable), "rtt_p95_ms": percentil(rtt_estable, 95),
                  "rtt_max_ms": max(rtt_estable), "rtt_n": len(rtt_estable)})

    print(f"Registro: {os.path.basename(ruta)}")
    print(f"Duración: {duracion:.1f} s  |  tramas válidas: {ok}")
    print(f"Frecuencia de recepción: {r['frecuencia_hz']:.2f} Hz (esperada 50 Hz)")
    print(f"Periodo de muestreo del ESP32: media {r['periodo_esp_ms']:.2f} ms "
          f"(mín {r['periodo_esp_min']:.2f}, máx {r['periodo_esp_max']:.2f})")
    print(f"Intervalo de llegada al PC: media {r['llegada_media_ms']:.2f} ms, jitter {r['jitter_ms']:.2f} ms, "
          f"p95 {r['llegada_p95_ms']:.2f} ms, máx {r['llegada_max_ms']:.2f} ms")
    print(f"Tramas perdidas: {perdidas} de {ok + perdidas} ({100 * perdidas / max(ok + perdidas, 1):.3f} %)"
          f"  |  checksum inválido: {malas}")
    if rtt_estable:
        print(f"RTT: mediana {r['rtt_mediana_ms']:.2f} ms, media {r['rtt_media_ms']:.2f} ms, "
              f"mín {r['rtt_min_ms']:.2f} ms, p95 {r['rtt_p95_ms']:.2f} ms, máx {r['rtt_max_ms']:.2f} ms "
              f"({r['rtt_n']} mediciones)")

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    ax[0].hist(llegada, bins=60, color="#2a78c2")
    ax[0].axvline(20, color="#d62728", ls="--", label="20 ms (50 Hz)")
    ax[0].set_title("Intervalo de llegada entre tramas consecutivas (PC)")
    ax[0].set_xlabel("ms")
    ax[0].set_ylabel("cantidad de tramas")
    ax[0].legend()
    if rtt_estable:
        ax[1].plot(t_rtt_estable, rtt_estable, "o-", color="#2ca02c", ms=3)
        ax[1].axhline(r["rtt_mediana_ms"], color="#555", ls="--", label=f"mediana {r['rtt_mediana_ms']:.1f} ms")
        ax[1].legend()
    ax[1].set_title("RTT PC → ESP32 → PC")
    ax[1].set_xlabel("tiempo (s)")
    ax[1].set_ylabel("ms")
    fig.suptitle(f"Validación de la comunicación – {ok} tramas, {perdidas} perdidas, {malas} con error")
    fig.tight_layout()
    salida = os.path.splitext(ruta)[0] + "_comunicacion.png"
    fig.savefig(salida, dpi=130)
    print(f"Figura guardada: {salida}")
    return r


def main():
    objetivo = sys.argv[1] if len(sys.argv) > 1 else "."
    if os.path.isdir(objetivo):
        registros = sorted(glob.glob(os.path.join(objetivo, "tramas_*.csv")))
        if not registros:
            print(f"No hay registros tramas_*.csv en {objetivo}")
            return
        objetivo = registros[-1]
    analizar(objetivo)


if __name__ == "__main__":
    main()
