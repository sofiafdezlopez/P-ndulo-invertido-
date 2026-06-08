/*
 * Ejemplo 1 — PIDControl en modo STANDARD (posicional)
 *
 * Equivalente al pseudocódigo:
 *   error := setpoint - measured_value
 *   integral := integral + error * dt
 *   derivative := (error - previous_error) / dt
 *   output := Kp*error + Ki*integral + Kd*derivative
 *   previous_error := error
 *
 * Circuito de prueba:
 *   - Potenciómetro en A0  → simula el sensor (measured_value)
 *   - PWM en pin 9         → salida del controlador (actuador)
 */

#include <PIDControl.h>

// Parámetros del controlador
const double Kp = 2.0;
const double Ki = 0.5;
const double Kd = 0.1;
const double dt = 0.01;   // 10 ms en segundos
const double u0 = 0.0;    // Valor inicial del actuador

// Setpoint deseado (0–1023 para lectura analógica de 10 bits)
const double SETPOINT = 512.0;

PIDControl pid(Kp, Ki, Kd, dt, u0, MODE_STANDARD);

void setup() {
    Serial.begin(115200);
    pid.setSetpoint(SETPOINT);
    pid.setOutputLimits(0, 255);   // Rango PWM de Arduino
}

void loop() {
    double sensor = analogRead(A0);           // Lectura del sensor
    double output = pid.compute(sensor);      // Calcular salida PID
    analogWrite(9, (int)output);              // Enviar al actuador

    Serial.print("Setpoint: "); Serial.print(SETPOINT);
    Serial.print("  Sensor: "); Serial.print(sensor);
    Serial.print("  Output: "); Serial.println(output);

    delay((unsigned long)(dt * 1000));        // Esperar dt ms
}
