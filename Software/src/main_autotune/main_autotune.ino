// ═══════════════════════════════════════════════════════════════════════════════
//  Péndulo Invertido — Control PID con Autotuning (Relay Feedback)
//  Método: Åström-Hägglund (1984)
//
//  Comandos Serial (115200 baud):
//    'A'  →  Forzar nuevo autotune
//    'Z'  →  Re-aplicar Ziegler-Nichols con el último Ku/Tu
//    'T'  →  Re-aplicar Tyreus-Luyben con el último Ku/Tu (más conservador)
//    'P'  →  Imprimir ganancias actuales
// ═══════════════════════════════════════════════════════════════════════════════

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Preferences.h"   // NVS — guarda ganancias en flash

MPU6050    sensor;
Preferences prefs;

// ─── Pines de motores ─────────────────────────────────────────────────────────
const int pinPWMA = 12, pinAIN2 = 33 pinAIN1 = 25;
const int pinPWMB = 15, pinBIN1 = 26, pinBIN2 = 27;

// ─── Límites de operación ─────────────────────────────────────────────────────
const int   PWM_MAX    = 170;   // 255 × (6 V / 9 V) — protege motores de 6 V
const int   DEADBAND   = 35;    // PWM mínimo para vencer la inercia del motor
const float FALL_ANGLE = 35.0f; // Si el robot supera ±35° respecto al equilibrio → parar

// ─── Ángulo de equilibrio ─────────────────────────────────────────────────────
//   Usa calibracion_mpu6050.cpp para medirlo y pon aquí el valor exacto.
//   Si el robot cae siempre hacia el mismo lado, prueba con el signo opuesto.
const float EQUILIBRIUM_ANGLE = 6.0f;  // ← Ajusta con tu calibración

// ─── Parámetros de filtrado ───────────────────────────────────────────────────
const float ALPHA_LP   = 0.12f;  // LP en acelerómetro (reduce vibración mecánica)
const float ALPHA_COMP = 0.98f;  // Peso giroscopio en filtro complementario
const float ALPHA_D    = 0.20f;  // LP en la derivada del PID (reduce amplificación de ruido)

// ─── Variables del sensor ────────────────────────────────────────────────────
int16_t ax, ay, az, gx, gy, gz;
float   gx_offset = 0, gy_offset = 0; // Bias del giroscopio en reposo
float   ax_f = 0, ay_f = 0, az_f = 0; // Acelerómetro filtrado (LP)
float   ang_y = 0, ang_y_prev = 0;    // Ángulo fusionado actual y anterior
long    tiempo_prev = 0;
float   dt = 0;

// ─── Ganancias PID ───────────────────────────────────────────────────────────
float Kp = 0, Ki = 0, Kd = 0;

// ─── Variables internas del PID ──────────────────────────────────────────────
float integral   = 0;
float prev_error = 0;
float filt_d     = 0;  // Derivada filtrada

// ─── Último resultado de identificación (para cambiar de fórmula sin re-tunear)
float last_Ku = -1, last_Tu = -1;

// ─── Estado del controlador ──────────────────────────────────────────────────
enum ControlState { STATE_AUTOTUNE, STATE_PID };
ControlState controlState = STATE_AUTOTUNE;

// ─── Parámetros del Autotune ─────────────────────────────────────────────────
//
//  RELAY_AMP: amplitud PWM del relay (entre DEADBAND y PWM_MAX).
//    - Demasiado bajo  → el robot no oscila, cae.
//    - Demasiado alto  → oscilaciones muy grandes, también cae.
//    - Buen punto de partida: ~110 (≈ 65 % de PWM_MAX).
//
//  CYCLES_TARGET: cuántos ciclos completos promediar antes de calcular.
//    - Más ciclos = estimación más precisa pero tarda más.
//    - 6-10 es suficiente en la práctica.
//
const float RELAY_AMP       = 110.0f;
const int   CYCLES_TARGET   = 8;
const long  AT_TIMEOUT_MS   = 30000L; // Reiniciar autotune si pasan 30 s sin oscilar

// Variables internas del autotune
float at_last_error      = 0.0f;
long  at_last_cross_ms   = 0L;
float at_sum_half_period = 0.0f;
float at_sum_peak        = 0.0f;
int   at_half_cycles     = 0;
float at_peak_cur        = 0.0f;  // Pico acumulado en el semiciclo actual
long  at_start_ms        = 0L;

// ─── Prototipos ───────────────────────────────────────────────────────────────
float readAngle();
void  handleAutotune(float error, float &output);
void  applyZieglerNichols(float Ku, float Tu);
void  applyTyreusLuyben(float Ku, float Tu);
void  saveGains();
void  printGains();
void  startPID();
void  resetAutotune();
void  moveMotor(int pwmPin, int in1, int in2, float speed);


// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Wire.begin(18, 19);
  Wire.setClock(400000);  // 400 kHz — máxima velocidad del MPU6050
  sensor.initialize();

  if (!sensor.testConnection()) {
    Serial.println("ERROR: MPU6050 no detectado. Revisa conexiones I2C.");
    while (true) delay(100);
  }

  // ── Configurar pines de motores ───────────────────────────────────────────
  int motorPins[] = {pinAIN1, pinAIN2, pinPWMA, pinBIN1, pinBIN2, pinPWMB};
  for (int p : motorPins) pinMode(p, OUTPUT);

  // ── Calibrar offset del giroscopio (500 muestras en reposo) ──────────────
  Serial.println("Calibrando giroscopio... deja el robot inmóvil.");
  long sgx = 0, sgy = 0;
  for (int i = 0; i < 500; i++) {
    sensor.getRotation(&gx, &gy, &gz);
    sgx += gx;
    sgy += gy;
    delay(3);
  }
  gx_offset = sgx / 500.0f;
  gy_offset = sgy / 500.0f;
  Serial.printf("  Offsets → GX: %.1f  GY: %.1f  (LSB)\n", gx_offset, gy_offset);

  // ── Intentar cargar ganancias guardadas en flash ──────────────────────────
  prefs.begin("pid", true);
  float saved = prefs.getFloat("Kp", -1.0f);
  prefs.end();

  if (saved > 0.0f) {
    prefs.begin("pid", true);
    Kp      = prefs.getFloat("Kp", 0);
    Ki      = prefs.getFloat("Ki", 0);
    Kd      = prefs.getFloat("Kd", 0);
    last_Ku = prefs.getFloat("Ku", -1);
    last_Tu = prefs.getFloat("Tu", -1);
    prefs.end();
    controlState = STATE_PID;
    Serial.println("Ganancias recuperadas de la flash:");
    printGains();
    Serial.println("Envía 'A' para forzar un nuevo autotune.");
  } else {
    Serial.println("\n══════════════════════════════════════════════════");
    Serial.println("  No hay ganancias guardadas → modo AUTOTUNE");
    Serial.println("  Pon el robot cerca del equilibrio y suéltalo.");
    Serial.println("══════════════════════════════════════════════════\n");
  }

  tiempo_prev = millis();
  at_start_ms = millis();
}


// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  // ── Comandos por Serial ──────────────────────────────────────────────────
  if (Serial.available()) {
    char c = toupper(Serial.read());
    switch (c) {
      case 'A':
        resetAutotune();
        controlState = STATE_AUTOTUNE;
        Serial.println("=== Autotune reiniciado. Pon el robot en equilibrio. ===");
        break;
      case 'Z':
        if (last_Ku > 0) { applyZieglerNichols(last_Ku, last_Tu); startPID(); }
        else Serial.println("Sin datos de autotune. Envía 'A' primero.");
        break;
      case 'T':
        if (last_Ku > 0) { applyTyreusLuyben(last_Ku, last_Tu); startPID(); }
        else Serial.println("Sin datos de autotune. Envía 'A' primero.");
        break;
      case 'P':
        printGains();
        break;
    }
  }

  // ── Calcular dt ──────────────────────────────────────────────────────────
  long now = millis();
  dt = constrain((now - tiempo_prev) / 1000.0f, 0.001f, 0.05f);
  tiempo_prev = now;

  // ── Leer ángulo fusionado ────────────────────────────────────────────────
  float angle = readAngle();
  float error = EQUILIBRIUM_ANGLE - angle;

  // ── Protección por caída ──────────────────────────────────────────────────
  //   Si el robot se aleja más de FALL_ANGLE del equilibrio, detenemos
  //   motores y reiniciamos el autotune (si estaba activo).
  if (fabsf(error) > FALL_ANGLE) {
    moveMotor(pinPWMA, pinAIN1, pinAIN2, 0);
    moveMotor(pinPWMB, pinBIN1, pinBIN2, 0);
    integral = 0;
    if (controlState == STATE_AUTOTUNE) {
      Serial.println("[AT] Caída detectada — reiniciando autotune.");
      resetAutotune();
    }
    return;
  }

  // ── Control (Autotune o PID) ──────────────────────────────────────────────
  float output = 0;

  if (controlState == STATE_AUTOTUNE) {
    handleAutotune(error, output);
  } else {
    // PID con antiwindup en integrador y filtro en derivada
    integral += error * dt;
    integral  = constrain(integral, -150.0f, 150.0f);

    float raw_d = (error - prev_error) / dt;
    filt_d      = ALPHA_D * raw_d + (1.0f - ALPHA_D) * filt_d;

    output = Kp * error + Ki * integral + Kd * filt_d;
  }
  prev_error = error;

  output = constrain(output, -(float)PWM_MAX, (float)PWM_MAX);
  moveMotor(pinPWMA, pinAIN1, pinAIN2, output);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, output);

  // ── Telemetría cada 50 ms ─────────────────────────────────────────────────
  static long lastPrint = 0;
  if (now - lastPrint >= 50) {
    lastPrint = now;
    const char* mode = (controlState == STATE_AUTOTUNE) ? "AT " : "PID";
    Serial.printf("[%s] ang=%6.2f  err=%6.2f  out=%5.0f\n",
                  mode, angle, error, output);
  }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  readAngle(): lee MPU6050 y devuelve el ángulo fusionado (filtro complementario)
// ═══════════════════════════════════════════════════════════════════════════════
float readAngle() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  // Filtro pasa-bajos en acelerómetro (atenúa vibraciones mecánicas)
  ax_f = ALPHA_LP * ax + (1.0f - ALPHA_LP) * ax_f;
  ay_f = ALPHA_LP * ay + (1.0f - ALPHA_LP) * ay_f;
  az_f = ALPHA_LP * az + (1.0f - ALPHA_LP) * az_f;

  // Ángulo del acelerómetro (referencia estática, sin deriva)
  float accel_ang = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * (180.0f / PI);

  // Velocidad angular corregida (sin bias de offset)
  float gy_dps = (gy - gy_offset) / 131.0f;  // 131 LSB/(°/s) para ±250°/s

  // Filtro complementario:  98 % giroscopio (dinámico) + 2 % acelerómetro (lento)
  ang_y      = ALPHA_COMP * (ang_y_prev + gy_dps * dt) + (1.0f - ALPHA_COMP) * accel_ang;
  ang_y_prev = ang_y;
  return ang_y;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  RELAY FEEDBACK AUTOTUNING (Åström & Hägglund, 1984)
//
//  Principio:
//    El relay fuerza oscilaciones sostenidas alrededor del punto de equilibrio.
//    Midiendo su periodo (Tu) y amplitud (a) se calcula la ganancia última:
//
//        Ku = (4 · d) / (π · a)
//
//    donde d = RELAY_AMP y a = amplitud media del error durante la oscilación.
//    A partir de Ku y Tu, las fórmulas de Z-N o T-L dan Kp, Ki y Kd.
//
//  Detección de oscilación:
//    Cada vez que el error cambia de signo (cruce por cero) se registra
//    el tiempo del semiciclo y el pico máximo de error de ese semiciclo.
//    Tras CYCLES_TARGET ciclos completos (2 × CYCLES_TARGET semiciclos)
//    se promedian y se calculan las ganancias.
// ═══════════════════════════════════════════════════════════════════════════════
void handleAutotune(float error, float &output) {
  // Relay puro: salida constante ±RELAY_AMP según el signo del error
  output = (error > 0.0f) ? RELAY_AMP : -RELAY_AMP;

  // ── Detectar cruce por cero (cambio de signo del error) ──────────────────
  bool crossed = (at_last_error * error < 0.0f);
  if (crossed) {
    long now = millis();

    if (at_last_cross_ms > 0) {
      float half_T = (now - at_last_cross_ms) / 1000.0f;

      // Filtrar semiciclos anómalos (< 50 ms o > 2 s → ruido o caída)
      if (half_T > 0.05f && half_T < 2.0f) {
        at_sum_half_period += half_T;
        at_sum_peak        += at_peak_cur;
        at_half_cycles++;

        Serial.printf("[AT] Semiciclo %-2d: T/2 = %.3f s  pico = %.2f°\n",
                      at_half_cycles, half_T, at_peak_cur);

        // ── ¿Suficientes ciclos? Calcular Ku y Tu ────────────────────────
        if (at_half_cycles >= 2 * CYCLES_TARGET) {
          float Tu = 2.0f * (at_sum_half_period / at_half_cycles);  // Periodo completo
          float a  = at_sum_peak / at_half_cycles;                   // Amplitud media
          float Ku = (4.0f * RELAY_AMP) / (PI * a);

          last_Ku = Ku;
          last_Tu = Tu;

          Serial.println("\n══════════════════════════════════════════════════");
          Serial.println("          AUTOTUNE COMPLETADO");
          Serial.println("══════════════════════════════════════════════════");
          Serial.printf("  Periodo último (Tu) = %.4f s\n", Tu);
          Serial.printf("  Ganancia última (Ku) = %.4f\n",  Ku);
          Serial.println("──────────────────────────────────────────────────");

          // Aplicar Ziegler-Nichols por defecto
          applyZieglerNichols(Ku, Tu);
          startPID();

          Serial.println("  Si el robot oscila demasiado → envía 'T' para");
          Serial.println("  cambiar a Tyreus-Luyben (más conservador).");
          Serial.println("══════════════════════════════════════════════════\n");
          return;
        }
      }
    }

    at_last_cross_ms = now;
    at_peak_cur      = 0.0f;  // Reiniciar pico para el nuevo semiciclo
  }

  // ── Acumular pico del semiciclo actual ────────────────────────────────────
  float abs_err = fabsf(error);
  if (abs_err > at_peak_cur) at_peak_cur = abs_err;

  // ── Timeout: sin oscilación → reintentar ──────────────────────────────────
  if (millis() - at_start_ms > AT_TIMEOUT_MS) {
    Serial.println("[AT] Timeout: sin oscilaciones en 30 s.");
    Serial.println("  → Sube RELAY_AMP, o acerca más el robot al equilibrio.");
    resetAutotune();
  }

  at_last_error = error;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  ZIEGLER-NICHOLS (1942)  —  Agresivo, buena respuesta transitoria
//
//    Kp = 0.60 · Ku
//    Ti = Tu / 2     →  Ki = Kp / Ti
//    Td = Tu / 8     →  Kd = Kp · Td
//
//  Úsala como punto de partida. Si el robot oscila en exceso, pasa a T-L.
// ═══════════════════════════════════════════════════════════════════════════════
void applyZieglerNichols(float Ku, float Tu) {
  Kp = 0.60f * Ku;
  Ki = Kp / (Tu / 2.0f);
  Kd = Kp * (Tu / 8.0f);
  Serial.println("  Regla aplicada: Ziegler-Nichols");
  printGains();
  saveGains();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  TYREUS-LUYBEN (1992)  —  Conservador, más robusto frente a no-linealidades
//
//    Kp = Ku / 3.2
//    Ti = 2.2 · Tu   →  Ki = Kp / Ti
//    Td = Tu / 6.3   →  Kd = Kp · Td
//
//  Recomendado si Z-N produce oscilaciones persistentes.
// ═══════════════════════════════════════════════════════════════════════════════
void applyTyreusLuyben(float Ku, float Tu) {
  Kp = Ku / 3.2f;
  Ki = Kp / (2.2f * Tu);
  Kd = Kp * (Tu / 6.3f);
  Serial.println("  Regla aplicada: Tyreus-Luyben");
  printGains();
  saveGains();
}


// ─── Guardar ganancias en NVS (flash no volátil del ESP32) ───────────────────
void saveGains() {
  prefs.begin("pid", false);
  prefs.putFloat("Kp", Kp);
  prefs.putFloat("Ki", Ki);
  prefs.putFloat("Kd", Kd);
  prefs.putFloat("Ku", last_Ku);
  prefs.putFloat("Tu", last_Tu);
  prefs.end();
  Serial.println("  Ganancias guardadas en flash (persisten al reiniciar).");
}

void printGains() {
  Serial.printf("  Kp = %.4f   Ki = %.4f   Kd = %.4f\n", Kp, Ki, Kd);
}

void startPID() {
  controlState = STATE_PID;
  integral = 0; prev_error = 0; filt_d = 0;
  Serial.println("  → Cambiando a modo PID.\n");
}

void resetAutotune() {
  at_half_cycles = 0; at_sum_half_period = 0; at_sum_peak = 0;
  at_last_cross_ms = 0; at_peak_cur = 0; at_last_error = 0;
  at_start_ms = millis();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  moveMotor(): aplica deadband, limita PWM y controla dirección
// ═══════════════════════════════════════════════════════════════════════════════
void moveMotor(int pwmPin, int in1, int in2, float speed) {
  int pwm = (int)fabsf(speed);
  if (pwm < DEADBAND) {
    // Por debajo del deadband: motor parado (evita corriente sin movimiento)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  } else {
    pwm = min(pwm, PWM_MAX);
    digitalWrite(in1, speed > 0 ? HIGH : LOW);
    digitalWrite(in2, speed > 0 ? LOW  : HIGH);
    analogWrite(pwmPin, pwm);
  }
}
