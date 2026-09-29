/*
 * Taller Segundo Corte - Consola de mandos ESP32 (real-to-sim)
 * Micros y Laboratorio - UMNG
 *
 * Firmware ÚNICO para los tres incisos del taller. La consola lee
 * 3 potenciómetros y 5 pulsadores y envía su estado completo por USB-Serial
 * (UART0, 115200 baudios) cada 20 ms (50 Hz). Cada script de Python decide
 * qué significa cada control (drones, Baxter con pinza, Atlas).
 *
 *   $<seq>,<t_ms>,<pot1>,<pot2>,<pot3>,<botones>,<c1>,<c2>,<c3>,<c4>,<c5>*<CS>\r\n
 *
 *   seq      contador de tramas (0..65535) -> detectar tramas perdidas
 *   t_ms     millis() del ESP32 al tomar la muestra
 *   pot1..3  lectura ADC filtrada, 12 bits (0..4095)
 *   botones  máscara de bits de los pulsadores PRESIONADOS en este instante
 *            (bit0 = pulsador 1 ... bit4 = pulsador 5)
 *   c1..c5   contador de pulsaciones de cada pulsador (0..65535)
 *   CS       checksum XOR (hex) de todos los caracteres entre '$' y '*'
 *
 * Responde "P<n>\n" con "#PONG,<n>" para medir el RTT desde el PC.
 */

#include <Arduino.h>
#include "esp_timer.h"

// ---------------- Pines (ESP32 DevKit 30 pines) ----------------
const uint8_t PINES_POT[3] = {34, 35, 32};          // ADC1
const uint8_t PINES_BTN[5] = {18, 19, 21, 22, 23};  // a GND, pull-up interno
const uint8_t PIN_LED = 2;                          // LED azul de la placa

// ---------------- Parámetros ----------------
const uint32_t PERIODO_US   = 20000;  // 20 ms -> 50 Hz
const uint8_t  MUESTRAS_ADC = 16;     // promedio por lectura
const float    ALFA_EMA     = 0.35f;  // filtro exponencial
const uint32_t REBOTE_MS    = 30;     // antirrebote
const uint32_t LED_MS       = 60;     // destello del LED al presionar

// ---------------- Temporizador de muestreo ----------------
volatile bool tocaMuestrear = false;
esp_timer_handle_t timerMuestreo;
void alVencerTimer(void *) { tocaMuestrear = true; }

// ---------------- Potenciómetros ----------------
float filtro[3] = {0, 0, 0};

uint16_t leerPromedio(uint8_t pin) {
  uint32_t suma = 0;
  for (uint8_t i = 0; i < MUESTRAS_ADC; i++) suma += analogRead(pin);
  return suma / MUESTRAS_ADC;
}

// ---------------- Pulsadores con antirrebote ----------------
struct Boton {
  uint8_t pin;
  bool estadoEstable;     // HIGH = suelto (pull-up)
  bool lecturaAnterior;
  uint32_t tCambio;
  uint16_t contador;      // número de pulsaciones
};
Boton botones[5];

// Actualiza el botón; devuelve true en el instante en que se presiona
bool actualizarBoton(Boton &b) {
  bool lectura = digitalRead(b.pin);
  if (lectura != b.lecturaAnterior) { b.tCambio = millis(); b.lecturaAnterior = lectura; }
  if ((millis() - b.tCambio) > REBOTE_MS && lectura != b.estadoEstable) {
    b.estadoEstable = lectura;
    if (lectura == LOW) { b.contador++; return true; }
  }
  return false;
}

uint16_t secuencia = 0;
uint32_t tLedApagar = 0;

// ---------------- Comandos del PC (medición de RTT) ----------------
char bufRx[24];
uint8_t idxRx = 0;

void atenderSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      bufRx[idxRx] = '\0';
      if (idxRx > 1 && bufRx[0] == 'P') Serial.printf("#PONG,%s\r\n", bufRx + 1);
      idxRx = 0;
    } else if (idxRx < sizeof(bufRx) - 1) {
      bufRx[idxRx++] = c;
    }
  }
}

// ---------------- Envío de la trama ----------------
void enviarTrama() {
  uint8_t mascara = 0;
  for (uint8_t i = 0; i < 5; i++) if (botones[i].estadoEstable == LOW) mascara |= (1 << i);

  char datos[96];
  int n = snprintf(datos, sizeof(datos), "%u,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u",
                   secuencia, (unsigned long)millis(),
                   (uint16_t)filtro[0], (uint16_t)filtro[1], (uint16_t)filtro[2], mascara,
                   botones[0].contador, botones[1].contador, botones[2].contador,
                   botones[3].contador, botones[4].contador);
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)datos[i];
  Serial.printf("$%s*%02X\r\n", datos, cs);
  secuencia++;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(PIN_LED, OUTPUT);

  for (uint8_t i = 0; i < 5; i++) {
    pinMode(PINES_BTN[i], INPUT_PULLUP);
    botones[i] = {PINES_BTN[i], HIGH, HIGH, 0, 0};
  }
  for (uint8_t i = 0; i < 3; i++) filtro[i] = leerPromedio(PINES_POT[i]);

  esp_timer_create_args_t args = {};
  args.callback = &alVencerTimer;
  args.name = "muestreo";
  esp_timer_create(&args, &timerMuestreo);
  esp_timer_start_periodic(timerMuestreo, PERIODO_US);

  Serial.println("#INFO Consola ESP32 - Taller Segundo Corte");
  Serial.println("#INFO Trama: $seq,t_ms,pot1,pot2,pot3,botones,c1,c2,c3,c4,c5*CS a 50 Hz");
}

void loop() {
  for (uint8_t i = 0; i < 5; i++) {
    if (actualizarBoton(botones[i])) {       // destello del LED como confirmación
      digitalWrite(PIN_LED, HIGH);
      tLedApagar = millis() + LED_MS;
    }
  }
  if (tLedApagar && (int32_t)(millis() - tLedApagar) >= 0) {
    digitalWrite(PIN_LED, LOW);
    tLedApagar = 0;
  }

  atenderSerial();

  if (tocaMuestrear) {
    tocaMuestrear = false;
    for (uint8_t i = 0; i < 3; i++) {
      filtro[i] = ALFA_EMA * leerPromedio(PINES_POT[i]) + (1.0f - ALFA_EMA) * filtro[i];
    }
    enviarTrama();
  }
}
