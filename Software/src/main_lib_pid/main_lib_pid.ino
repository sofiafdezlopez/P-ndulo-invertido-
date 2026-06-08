/*
 * main_pid_lib.ino — Prueba de los tres modos PIDControl en el péndulo invertido
 *
 * Práctica 4, sección 4.2: "Realizar pruebas de funcionamiento de los
 * controladores PID implementados en el péndulo invertido construido."
 *
 * Comandos por Serial (115200 baud):
 *   'S' → MODE_STANDARD  (PID posicional clásico)
 *   'D' → MODE_DISCRETE  (IIR, tres errores sin acumulador)
 *   'F' → MODE_FILTERED  (PI + derivada con filtro pasa-baja)  ← por defecto
 *
 * Qué observar al cambiar de modo:
 *   STANDARD  → señal de control más ruidosa, puede vibrar más
 *   DISCRETE  → sin integral acumulado explícito; puede derivar si Kp/Ki son
 *               pequeños (la ecuación asume acumulación u[k-1], ver nota)
 *   FILTERED  → salida más suave, mejor respuesta en robot real con MPU6050
 *
 * Nota sobre MODE_DISCRETE: el pseudocódigo del guión calcula
 *   output = A0·e[k] + A1·e[k-1] + A2·e[k-2]
 * sin acumular el u[k-1] anterior. Esto sigue siendo válido para comparar
 * comportamientos, pero matemáticamente es la forma incremental (Δu).
 */

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PIDControl.h>

// ── Pines de motores ──────────────────────────────────────────────────────────
const int pinPWMA = 12, pinAIN1 = 33, pinAIN2 = 25;
const int pinPWMB = 15, pinBIN1 = 26, pinBIN2 = 27;

// ── Límites PWM ───────────────────────────────────────────────────────────────
const int PWM_MAX  = 170;  // 255 × (6V / 9V) — protege motores de 6V

// ── Ángulo de equilibrio y límite de caída ────────────────────────────────────
const float EQUILIBRIUM_ANGLE = 6.0f;   // Ajustar con calibracion_mpu6050
const float FALL_ANGLE        = 24.0f;  // Más de ±35° → está caído

// ── Filtros del sensor ────────────────────────────────────────────────────────
const float ALPHA_LP   = 0.12f;  // Paso-bajo en acelerómetro
const float ALPHA_COMP = 0.98f;  // Peso del giroscopio en filtro complementario

// ── Período de muestreo ───────────────────────────────────────────────────────
// Debe coincidir con el dt usado en el constructor de PIDControl.
// El bucle se sincroniza con delay() al final para cumplirlo.
const double DT_S = 0.01;  // 10 ms

// ── Ganancias PID ─────────────────────────────────────────────────────────────
// Ajustar según sintonización. Punto de partida conservador:
const double KP = 35.0;
const double KI = 0.1;
const double KD = 5.0;

// ── Tres controladores, uno por modo ─────────────────────────────────────────
// Se crean todos a la vez para poder cambiar de modo sin reiniciar el sketch.
PIDControl pidS(KP, KI, KD, DT_S, 0.0, MODE_STANDARD);
PIDControl pidD(KP, KI, KD, DT_S, 0.0, MODE_DISCRETE);
PIDControl pidF(KP, KI, KD, DT_S, 0.0, MODE_FILTERED);
PIDControl*  activePid  = &pidF;
PIDMode      currentMode = MODE_FILTERED;

// ── Sensor ────────────────────────────────────────────────────────────────────
MPU6050 sensor;
int16_t ax, ay, az, gx, gy, gz;
float   gy_offset = 0.0f;
float   ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
float   ang_y = 0.0f, ang_y_prev = 0.0f;

// ── Prototipo ─────────────────────────────────────────────────────────────────
void moveMotor(int pwmPin, int in1, int in2, float speed);
void switchMode(PIDMode mode, const char* name);


// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Wire.begin(18, 19);
    Wire.setClock(400000);
    sensor.initialize();

    if (!sensor.testConnection()) {
        Serial.println("ERROR: MPU6050 no detectado. Revisa el cableado I2C.");
        while (true) delay(100);
    }

    // Configurar pines de motores
    const int motorPins[] = {pinAIN1, pinAIN2, pinPWMA, pinBIN1, pinBIN2, pinPWMB};
    for (int i = 0; i < 6; i++) pinMode(motorPins[i], OUTPUT);

    // Parar motores al arrancar
    moveMotor(pinPWMA, pinAIN1, pinAIN2, 0);
    moveMotor(pinPWMB, pinBIN1, pinBIN2, 0);

    // Calibrar offset del giroscopio (500 muestras en reposo)
    Serial.println("Calibrando giroscopio... deja el robot inmovil.");
    long sg = 0;
    for (int i = 0; i < 500; i++) {
        sensor.getRotation(&gx, &gy, &gz);
        sg += gy;
        delay(3);
    }
    gy_offset = sg / 500.0f;
    Serial.printf("  Offset GY = %.1f LSB\n", gy_offset);

    // Configurar setpoint y límites en los tres controladores
    pidS.setSetpoint(EQUILIBRIUM_ANGLE);  pidS.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidD.setSetpoint(EQUILIBRIUM_ANGLE);  pidD.setOutputLimits(-PWM_MAX, PWM_MAX);
    pidF.setSetpoint(EQUILIBRIUM_ANGLE);  pidF.setOutputLimits(-PWM_MAX, PWM_MAX);

    // Ángulo inicial desde el acelerómetro
    sensor.getAcceleration(&ax, &ay, &az);
    ang_y = atan2f(-(float)ax,
                   sqrtf((float)ay*(float)ay + (float)az*(float)az)) * (180.0f / PI);
    ang_y_prev = ang_y;

    Serial.println("\n=== PRUEBA MODOS PIDControl — Pendulo Invertido ===");
    Serial.printf("  KP=%.2f  KI=%.2f  KD=%.2f  DT=%dms\n",
                  (float)KP, (float)KI, (float)KD, (int)(DT_S * 1000));
    Serial.println("  S → MODE_STANDARD   D → MODE_DISCRETE   F → MODE_FILTERED");
    Serial.println("  Modo activo: FILTERED\n");
}


// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    unsigned long t0 = millis();

    // ── Comandos por Serial ───────────────────────────────────────────────────
    if (Serial.available()) {
        char c = (char)toupper(Serial.read());
        if      (c == 'S') switchMode(MODE_STANDARD, "STANDARD");
        else if (c == 'D') switchMode(MODE_DISCRETE, "DISCRETE");
        else if (c == 'F') switchMode(MODE_FILTERED, "FILTERED");
    }

    // ── Lectura y filtrado del sensor ─────────────────────────────────────────
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);

    // Filtro paso-bajo en acelerómetro (atenúa vibraciones mecánicas)
    ax_f = ALPHA_LP*(float)ax + (1.0f - ALPHA_LP)*ax_f;
    ay_f = ALPHA_LP*(float)ay + (1.0f - ALPHA_LP)*ay_f;
    az_f = ALPHA_LP*(float)az + (1.0f - ALPHA_LP)*az_f;

    // Ángulo desde acelerómetro filtrado
    float accel_ang = atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)) * (180.0f / PI);

    // Velocidad angular corregida con offset
    float gy_dps = ((float)gy - gy_offset) / 131.0f;

    // Filtro complementario
    ang_y      = ALPHA_COMP * (ang_y_prev + gy_dps * (float)DT_S)
               + (1.0f - ALPHA_COMP) * accel_ang;
    ang_y_prev = ang_y;

    float error = EQUILIBRIUM_ANGLE - ang_y;

    // ── Protección por caída ──────────────────────────────────────────────────
    if (fabsf(error) > FALL_ANGLE) {
        moveMotor(pinPWMA, pinAIN1, pinAIN2, 0);
        moveMotor(pinPWMB, pinBIN1, pinBIN2, 0);
        // Resetear los tres para evitar acumulación al recuperar
        pidS.reset();
        pidD.reset();
        pidF.reset();
        return;
    }

    // ── Calcular salida con el modo activo ────────────────────────────────────
    float output = (float)activePid->compute((double)ang_y);

    moveMotor(pinPWMA, pinAIN1, pinAIN2,  output);
    moveMotor(pinPWMB, pinBIN1, pinBIN2,  output);

    // ── Telemetría cada 50 ms ─────────────────────────────────────────────────
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    if (now - lastPrint >= 50) {
        lastPrint = now;
        const char* mStr = (currentMode == MODE_STANDARD) ? "STD" :
                           (currentMode == MODE_DISCRETE)  ? "DIS" : "FIL";
        Serial.printf("[%s] ang=%6.2f  err=%6.2f  out=%5.0f\n",
                      mStr, ang_y, error, output);
    }

    // ── Sincronizar bucle a DT fijo ───────────────────────────────────────────
    // El DT del constructor de PIDControl debe coincidir con el tiempo real de loop.
    long elapsed = (long)(millis() - t0);
    if (elapsed < 10) delay(10 - elapsed);
}


// ── Cambiar de modo: resetea todos los controladores antes de activar ─────────
void switchMode(PIDMode mode, const char* name) {
    pidS.reset();
    pidD.reset();
    pidF.reset();
    currentMode = mode;
    activePid = (mode == MODE_STANDARD) ? &pidS :
                (mode == MODE_DISCRETE)  ? &pidD : &pidF;
    Serial.printf(">>> Modo cambiado: MODE_%s\n", name);
}


// ── Control de motores con límite PWM ────────────────────────────────────────
void moveMotor(int pwmPin, int in1, int in2, float speed) {
    int pwm = (int)fabsf(speed);
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    digitalWrite(in1, speed > 0.0f ? HIGH : LOW);
    digitalWrite(in2, speed > 0.0f ? LOW  : HIGH);
    analogWrite(pwmPin, pwm);
}