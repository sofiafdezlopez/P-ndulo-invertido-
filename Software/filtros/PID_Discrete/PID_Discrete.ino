/*
 * Ejemplo 2 — PIDControl en modo DISCRETE (forma de velocidad)
 *
 * Equivalente al pseudocódigo:
 *   A0 = Kp + Ki*dt + Kd/dt
 *   A1 = -Kp - 2*Kd/dt
 *   A2 = Kd/dt
 *   output := A0*e[0] + A1*e[1] + A2*e[2]
 *
 * Ventaja frente al modo STANDARD: calcula la salida directamente
 * desde los tres errores más recientes sin acumulador de integrador.
 */

#include <PIDControl.h>

const double Kp = 2.0;
const double Ki = 0.5;
const double Kd = 0.1;
const double dt = 0.01;
const double u0 = 0.0;

const double SETPOINT = 512.0;

PIDControl pid(Kp, Ki, Kd, dt, u0, MODE_DISCRETE);

void setup() {
    Serial.begin(115200);
    pid.setSetpoint(SETPOINT);
    pid.setOutputLimits(0, 255);
}

void loop() {
    double sensor = analogRead(A0);
    double output = pid.compute(sensor);
    analogWrite(9, (int)output);

    Serial.print("Setpoint: "); Serial.print(SETPOINT);
    Serial.print("  Sensor: "); Serial.print(sensor);
    Serial.print("  Output: "); Serial.println(output);

    delay((unsigned long)(dt * 1000));
}
