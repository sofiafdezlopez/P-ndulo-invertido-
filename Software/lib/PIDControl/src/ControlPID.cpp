#include "Arduino.h"
#include "PIDControl.h"

// Determina los coeficientes necesarios para cada modo de operación
PIDControl::PIDControl(double Kp, double Ki, double Kd, double dt,
                       double u0, PIDMode mode)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _dt(dt),
      _setpoint(0.0), _mode(mode),
      _output(u0), _limitOutput(false), _outMin(0.0), _outMax(0.0),
      _prevError(0.0), _integral(0.0),
      _d0(0.0), _d1(0.0), _fd0(0.0), _fd1(0.0)
{
    _error[0] = _error[1] = _error[2] = 0.0;

    // Coeficientes para MODE_DISCRETE
    // Forma discreta: output = A0*e[k] + A1*e[k-1] + A2*e[k-2]
    _A0 = Kp + Ki * dt + Kd / dt;
    _A1 = -Kp - 2.0 * Kd / dt;
    _A2 = Kd / dt;

    // Coeficientes para MODE_FILTERED - parte PI incremental
    // PI: output += A0pi*e[k] + A1pi*e[k-1]
    _A0pi = Kp + Ki * dt;
    _A1pi = -Kp;

    // Coeficientes para MODE_FILTERED - parte derivativa sin filtrar
    // D: d0 = A0d*e[k] + A1d*e[k-1] + A2d*e[k-2]
    _A0d = Kd / dt;
    _A1d = -2.0 * Kd / dt;
    _A2d = Kd / dt;

    // Filtro bilineal paso-bajo: atenúa el ruido del derivativo
    // N es el orden del filtro (5 es estándar)
    // tau es la constante de tiempo = Kd / (Kp * N)
    const int N  = 5;
    double tau   = (Kp > 0.0) ? (Kd / (Kp * N)) : 1e-6;
    double alpha = dt / (2.0 * tau);
    _alpha1      = alpha / (alpha + 1.0);
    _alpha2      = (alpha - 1.0) / (alpha + 1.0);
}

// Establece los límites de la salida
void PIDControl::establecerLimiteSalida(double min, double max) {
    if (min >= max) return;
    _limitOutput = true;
    _outMin = min;
    _outMax = max;
    _output = _clamp(_output);
}

// Establece el valor objetivo (setpoint)
void PIDControl::establecerSetpoint(double setpoint) {
    _setpoint = setpoint;
}

// Reinicia el estado interno: borra integral, errores anteriores y filtros
void PIDControl::reiniciar() {
    _prevError = 0.0;
    _integral  = 0.0;
    _error[0] = _error[1] = _error[2] = 0.0;
    _d0 = _d1 = _fd0 = _fd1 = 0.0;
}

// Calcula la salida del controlador según el modo actual
// Retorna la acción de control limitada entre _outMin y _outMax
double PIDControl::calcular(double valor_medido) {
    switch (_mode) {

        // Modo clásico: error acumulado + derivada instantánea
        case MODE_STANDARD: {
            double error      = _setpoint - valor_medido;
            _integral        += error * _dt;          // Acumular integral
            double derivative = (error - _prevError) / _dt;  // Cambio de error
            _output           = _Kp * error + _Ki * _integral + _Kd * derivative;
            _prevError        = error;                // Guardar para próxima iteración
            break;
        }

        // Modo discreto: calcula directamente desde los 3 últimos errores
        case MODE_DISCRETE: {
            _error[2] = _error[1];      // Desplazar buffer: e[k-2] ← e[k-1]
            _error[1] = _error[0];      // e[k-1] ← e[k]
            _error[0] = _setpoint - valor_medido;  // Nuevo error e[k]
            _output   = _A0 * _error[0] + _A1 * _error[1] + _A2 * _error[2];
            break;
        }

        // Modo filtrado: PI incremental + derivada con filtro pasa-baja
        case MODE_FILTERED: {
            _error[2] = _error[1];
            _error[1] = _error[0];
            _error[0] = _setpoint - valor_medido;

            // Parte PI incremental (se suma a la salida anterior)
            _output += _A0pi * _error[0] + _A1pi * _error[1];

            // Parte derivativa con filtro bilineal (atenúa ruido)
            _d1  = _d0;  // Guardar valor anterior
            _d0  = _A0d * _error[0] + _A1d * _error[1] + _A2d * _error[2];
            _fd1 = _fd0;  // Guardar salida filtrada anterior
            // Filtro: salida = alpha1*(d0+d1) - alpha2*fd1
            _fd0 = _alpha1 * (_d0 + _d1) - _alpha2 * _fd1;
            _output += _fd0;  // Añadir parte filtrada a la salida
            break;
        }
    }

    // Limitar la salida entre los límites establecidos
    _output = _clamp(_output);
    return _output;
}

// Función auxiliar: limita un valor entre _outMin y _outMax
// Si los límites no están establecidos, retorna el valor sin cambios
    if (!_limitOutput) return value;
    if (value > _outMax) return _outMax;
    if (value < _outMin) return _outMin;
    return value;
}