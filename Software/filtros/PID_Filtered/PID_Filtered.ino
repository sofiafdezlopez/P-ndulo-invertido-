/*
 * Ejemplo 3 — PIDControl en modo FILTERED
 * (derivativo con filtro pasa-baja, sección 4.1.3 del guión)
 *
 * El término derivativo puro amplifica el ruido de alta frecuencia.
 * Para atenuarlo se añade un filtro pasa-baja de primer orden con
 * constante de tiempo  τ = Kd / (Kp · N),  con 3 ≤ N ≤ 10.
 *
 * Pseudocódigo equivalente (guión, sección 4.1.3):
 *
 *   A0  = Kp + Ki*dt          // PI incremental
 *   A1  = -Kp
 *   A0d = Kd/dt               // Derivativo sin filtrar
 *   A1d = -2·Kd/dt
 *   A2d = Kd/dt
 *   tau    = Kd / (Kp·N)
 *   alpha  = dt / (2·tau)
 *   alpha1 = alpha / (alpha + 1)
 *   alpha2 = (alpha - 1) / (alpha + 1)
 *
 *   loop:
 *     e[2] = e[1]; e[1] = e[0]; e[0] = setpoint - medida
 *     output += A0·e[0] + A1·e[1]          // PI
 *     d0  = A0d·e[0] + A1d·e[1] + A2d·e[2]
 *     fd0 = alpha1·(d0 + d1) - alpha2·fd1  // D filtrada
 *     output += fd0
 *
 * Circuito de prueba:
 *   - Potenciómetro en A0  → simula el sensor (measured_value)
 *   - PWM en pin 9         → salida del controlador (actuador)
 */

#include <PIDControl.h>

// ── Parámetros del controlador ─────────────────────────────────────────────
const double Kp = 2.0;
const double Ki = 0.5;
const double Kd = 0.1;
const double dt = 0.01;   // 10 ms en segundos
const double u0 = 0.0;   // Valor inicial del actuador

// Setpoint (0–1023 para ADC de 10 bits)
const double SETPOINT = 512.0;

// ── Instancia del controlador en modo FILTERED ─────────────────────────────
PIDControl pid(Kp, Ki, Kd, dt, u0, MODE_FILTERED);

void setup() {
    Serial.begin(115200);
    pid.setSetpoint(SETPOINT);
    pid.setOutputLimits(0, 255);   // Rango PWM de Arduino/ESP32
}

void loop() {
    double sensor = analogRead(A0);           // Lectura del sensor
    double output = pid.compute(sensor);      // Calcular salida PID filtrada
    analogWrite(9, (int)output);              // Enviar al actuador

    // Telemetría por Serial (útil para Serial Plotter)
    Serial.print("Setpoint: "); Serial.print(SETPOINT);
    Serial.print("  Sensor: "); Serial.print(sensor);
    Serial.print("  Output: "); Serial.println(output);

    delay((unsigned long)(dt * 1000));        // Esperar dt ms
}
