/*
 * main_lib_pid.ino - Prueba de los tres modos PID en el péndulo invertido
 * Comandos por Serial (115200):
 *   S -> MODE_STANDARD (PID clásico)
 *   D -> MODE_DISCRETE (forma de velocidad)
 *   F -> MODE_FILTERED (con filtro en derivada)
 */

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PIDControl.h>

// Pines de motores
const int pinPWMA = 12, pinAIN1 = 25, pinAIN2 = 33;
const int pinPWMB = 15, pinBIN1 = 26, pinBIN2 = 27;

// PWM máximo permitido (6V motores, 9V batería)
const int PWM_MAX = 170;

// Ángulo de equilibrio y límite de caída
const float EQUILIBRIUM_ANGLE = 0.0f;
const float FALL_ANGLE = 24.0f;

// Filtro complementario
const float ALPHA_COMP = 0.98f;

// Período de muestreo
const double DT_S = 0.01;

// Ganancias PID
const double KP = 30.0;
const double KI = 0.05;
const double KD = 15.0;

// Controladores PID para los tres modos
PIDControl pidS(KP, KI, KD, DT_S, 0.0, MODE_STANDARD);
PIDControl pidD(KP, KI, KD, DT_S, 0.0, MODE_DISCRETE);
PIDControl pidF(KP, KI, KD, DT_S, 0.0, MODE_FILTERED);
PIDControl* activePid = &pidF;
PIDMode currentMode = MODE_FILTERED;

// Sensor MPU6050
MPU6050 sensor;
int16_t ax, ay, az, gx, gy, gz;
float gy_offset = 0.0f;
float ang_y = 0.0f, ang_y_prev = 0.0f;

// Prototipos de funciones
void moveMotor(int pwmPin, int in1, int in2, float speed);
void switchMode(PIDMode mode, const char* name);


// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Wire.begin(18, 19);
    Wire.setClock(400000);
    sensor.initialize();

    // Calibración del MPU6050
    sensor.setXAccelOffset(-736);
    sensor.setYAccelOffset(1299);
    sensor.setZAccelOffset(1522);
    sensor.setXGyroOffset(166);
    sensor.setYGyroOffset(250);
    sensor.setZGyroOffset(56);

    if (!sensor.testConnection()) {
        Serial.println("ERROR: MPU6050 no detectado. Revisa el cableado I2C.");
        while (true) delay(100);
    }

    // Configurar pines de motores
    const int motorPins[] = {pinAIN1, pinAIN2, pinPWMA, pinBIN1, pinBIN2, pinPWMB};
    for (int i = 0; i < 6; i++) {
        pinMode(motorPins[i], OUTPUT);
    }

    moveMotor(pinPWMA, pinAIN1, pinAIN2, 0);
    moveMotor(pinPWMB, pinBIN1, pinBIN2, 0);

    // Configurar setpoint y límites
    pidS.establecerSetpoint(EQUILIBRIUM_ANGLE);
    pidS.establecerLimiteSalida(-PWM_MAX, PWM_MAX);
    pidD.establecerSetpoint(EQUILIBRIUM_ANGLE);
    pidD.establecerLimiteSalida(-PWM_MAX, PWM_MAX);
    pidF.establecerSetpoint(EQUILIBRIUM_ANGLE);
    pidF.establecerLimiteSalida(-PWM_MAX, PWM_MAX);

    // Ángulo inicial
    sensor.getAcceleration(&ax, &ay, &az);
    ang_y = atan2f(-(float)ax, sqrtf((float)ay*(float)ay + (float)az*(float)az)) * (180.0f / PI);
    ang_y_prev = ang_y;

    Serial.println("PIDControl - Pendulo Invertido");
    Serial.printf("KP=%.2f KI=%.2f KD=%.2f DT=%dms\n",
                  (float)KP, (float)KI, (float)KD, (int)(DT_S * 1000));
    Serial.println("Comandos: S=STANDARD D=DISCRETE F=FILTERED");
}


// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    unsigned long t0 = millis();

    // Comandos por Serial
    if (Serial.available()) {
        char command = (char)toupper(Serial.read());
        if (command == 'S') {
            switchMode(MODE_STANDARD, "STANDARD");
        } else if (command == 'D') {
            switchMode(MODE_DISCRETE, "DISCRETE");
        } else if (command == 'F') {
            switchMode(MODE_FILTERED, "FILTERED");
        }
    }

    // Leer sensor
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);

    // Calcular ángulo desde acelerómetro
    float accel_ang = atan2f(-(float)ax, sqrtf((float)ay*(float)ay + (float)az*(float)az)) * (180.0f / PI);

    // Velocidad angular
    float gy_dps = ((float)gy - gy_offset) / 131.0f;

    // Filtro complementario
    ang_y = ALPHA_COMP * (ang_y_prev + gy_dps * (float)DT_S) + (1.0f - ALPHA_COMP) * accel_ang;
    ang_y_prev = ang_y;

    float error = EQUILIBRIUM_ANGLE - ang_y;

    // Protección por caída
    if (fabsf(error) > FALL_ANGLE) {
        moveMotor(pinPWMA, pinAIN1, pinAIN2, 0);
        moveMotor(pinPWMB, pinBIN1, pinBIN2, 0);
        pidS.reiniciar();
        pidD.reiniciar();
        pidF.reiniciar();
        return;
    }

    // Calcular salida PID
    float output = (float)activePid->calcular((double)ang_y);

    moveMotor(pinPWMA, pinAIN1, pinAIN2, output);
    moveMotor(pinPWMB, pinBIN1, pinBIN2, -output);

    // Telemetría
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    if (now - lastPrint >= 50) {
        lastPrint = now;
        const char* modeStr;
        if (currentMode == MODE_STANDARD) {
            modeStr = "STD";
        } else if (currentMode == MODE_DISCRETE) {
            modeStr = "DIS";
        } else {
            modeStr = "FIL";
        }
        Serial.printf("[%s] ang=%6.2f err=%6.2f out=%5.0f\n", modeStr, ang_y, error, output);
    }

    // Sincronizar a DT fijo
    long elapsed = (long)(millis() - t0);
    if (elapsed < 10) {
        delay(10 - elapsed);
    }
}


void switchMode(PIDMode mode, const char* name) {
    pidS.reiniciar();
    pidD.reiniciar();
    pidF.reiniciar();
    currentMode = mode;

    if (mode == MODE_STANDARD) {
        activePid = &pidS;
    } else if (mode == MODE_DISCRETE) {
        activePid = &pidD;
    } else {
        activePid = &pidF;
    }
    Serial.printf("Modo: %s\n", name);
}


void moveMotor(int pwmPin, int in1, int in2, float speed) {
    int pwm = (int)fabsf(speed);

    if (pwm > PWM_MAX) {
        pwm = PWM_MAX;
    }

    if (speed > 0.0f) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }

    analogWrite(pwmPin, pwm);
}