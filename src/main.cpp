// src/main.cpp
#include <Arduino.h>
#include <ESP32Servo.h>

// ================= Pines =================
const int PIN_SENSOR = 34;
const int PIN_SERVO  = 27;

// ================= Parámetros PID (continuo) =================
// u = Kp*e + Ki*integral(e dt) + Kd*de/dt   [grados]
float Kp = 0.879;   // grados / cm
float Ki = 0.678;   // grados / (cm*s)
float Kd = 0.01;     // grados*s / cm

float SIGNO = 1.0;  // cambia a -1 si el servo corrige al revés

// ================= Referencia =================
// La posición se mide desde el extremo de la viga: 0 cm = 10 cm del sensor.
const float REF_MIN = 0.0;    // cm (posición)
const float REF_MAX = 38.5;   // cm (= 44.5 - 10 del sensor)
float referencia = 30.0;      // cm (posición; equivale a los 27 cm del sensor)

// ================= Servo =================
const float ANGULO_CENTRO = 90.0;
const float ANGULO_MIN    = 40.0;
const float ANGULO_MAX    = 135.0;
const float U_MIN = ANGULO_MIN - ANGULO_CENTRO;
const float U_MAX = ANGULO_MAX - ANGULO_CENTRO;

// ================= Calibración del sensor =================
// Distancia cruda del sensor: 10 cm (ADC_MAX) a 44.5 cm (ADC_MIN).
// La posición que usa el control es: posición = distancia_sensor - DISTANCIA_MIN,
// es decir, 0 cm cuando el sensor marca 10 cm.
const float DISTANCIA_MIN = 10.0;
const float DISTANCIA_MAX = 44.5;
const float ADC_MIN = 900.0;
const float ADC_MAX = 3060.0;

// ================= Filtro =================
const float TAU_FILTRO = 0.08;  // s, filtro pasa-bajos de la medida
const int   N_MUESTRAS = 16;    // lecturas ADC promediadas

// ================= Tiempos =================
const unsigned long PERIODO_CTRL_US = 20000;   // control: 50 Hz
const unsigned long PERIODO_PLOT_US = 50000;   // gráfica: 20 Hz
unsigned long tCtrl = 0;
unsigned long tPlot = 0;

// ================= Estados =================
float terminoI = 0.0;     // ya incluye Ki (en grados)
float distFilt = 0.0;
float distFilt_1 = 0.0;
float error = 0.0, angulo = ANGULO_CENTRO;
bool primeraVez = true;

Servo servo;

// ---------- Servo con resolución continua (microsegundos) ----------
void escribirServo(float a) {
  int us = (int)(500.0 + a * (2400.0 - 500.0) / 180.0);
  servo.writeMicroseconds(us);
}

// ---------- Comandos por Serial ----------
//   30        -> referencia = 30 cm (0 a 40)
//   r 30      -> igual
//   kp 0.25 | ki 0.05 | kd 0.1
//   s -1      -> signo
//   reset     -> reinicia la integral
void leerSerial() {
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  linea.toLowerCase();
  if (linea.length() == 0) return;

  if (linea == "reset") {
    terminoI = 0.0;
    Serial.println("# integral reiniciada");
    return;
  }

  char c = linea.charAt(0);
  if (isDigit(c) || c == '-' || c == '.') {
    referencia = constrain(linea.toFloat(), REF_MIN, REF_MAX);
    Serial.printf("# ref = %.2f\n", referencia);
    return;
  }

  int sp = linea.indexOf(' ');
  if (sp < 0) sp = linea.indexOf('=');
  if (sp < 0) { Serial.println("# comando no valido"); return; }

  String cmd = linea.substring(0, sp);
  float val = linea.substring(sp + 1).toFloat();

  if (cmd == "r") {
    referencia = constrain(val, REF_MIN, REF_MAX);
    Serial.printf("# ref = %.2f\n", referencia);
  } else if (cmd == "kp") {
    Kp = val; Serial.printf("# Kp = %.4f\n", Kp);
  } else if (cmd == "ki") {
    if (Ki != 0.0) terminoI *= (val / Ki); else terminoI = 0.0;
    Ki = val; Serial.printf("# Ki = %.4f\n", Ki);
  } else if (cmd == "kd") {
    Kd = val; Serial.printf("# Kd = %.4f\n", Kd);
  } else if (cmd == "s") {
    SIGNO = (val < 0) ? -1.0 : 1.0; Serial.printf("# signo = %.0f\n", SIGNO);
  } else {
    Serial.println("# comando no valido");
  }
}

// ---------- Envío a Teleplot (una sola escritura, sin bloquear) ----------
void enviarPlot() {
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
                   ">distancia_cm:%.2f\n>referencia:%.2f\n>error:%.2f\n>servo:%.2f\n",
                   distFilt, referencia, error, angulo);
  // Si el buffer de TX no tiene espacio, se omite esta muestra
  // en lugar de frenar el lazo de control.
  if (n > 0 && Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t*)buf, n);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR, ADC_11db);

  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);
  escribirServo(ANGULO_CENTRO);

  delay(1000);
  tCtrl = micros();
  tPlot = micros();
}

void loop() {
  leerSerial();

  unsigned long ahora = micros();
  if (ahora - tCtrl < PERIODO_CTRL_US) return;

  float dt = (ahora - tCtrl) / 1e6f;   // dt real medido
  tCtrl = ahora;

  // ---------- Lectura ----------
  long suma = 0;
  for (int i = 0; i < N_MUESTRAS; i++) suma += analogRead(PIN_SENSOR);
  float adc = suma / (float)N_MUESTRAS;

  // Posición de la bola: 0 cm (ADC_MAX, 10 cm del sensor) a 34.5 cm (ADC_MIN)
  float distancia =
      (ADC_MAX - adc) * (DISTANCIA_MAX - DISTANCIA_MIN) / (ADC_MAX - ADC_MIN);
  distancia = constrain(distancia, 0.0f, DISTANCIA_MAX - DISTANCIA_MIN);

  if (primeraVez) {
    distFilt = distancia;
    distFilt_1 = distancia;
    primeraVez = false;
  } else {
    float a = dt / (TAU_FILTRO + dt);
    distFilt += a * (distancia - distFilt);
  }

  // ---------- PID continuo ----------
  error = SIGNO * (referencia - distFilt);

  float P = Kp * error;

  // Derivativo sobre la medida (sin patada al cambiar la referencia)
  float derivada = SIGNO * (distFilt - distFilt_1) / dt;
  float D = -Kd * derivada;
  distFilt_1 = distFilt;

  float uSinI = P + D;

  // Integral con anti-windup por integración condicional
  float uTentativa = uSinI + terminoI;
  bool satAlto = (uTentativa >= U_MAX) && (error > 0);
  bool satBajo = (uTentativa <= U_MIN) && (error < 0);
  if (!satAlto && !satBajo) terminoI += Ki * error * dt;
  terminoI = constrain(terminoI, U_MIN, U_MAX);

  float u = constrain(uSinI + terminoI, U_MIN, U_MAX);
  angulo = ANGULO_CENTRO + u;
  escribirServo(angulo);

  // ---------- Gráfica a menor frecuencia que el control ----------
  if (ahora - tPlot >= PERIODO_PLOT_US) {
    tPlot = ahora;
    enviarPlot();
  }
}