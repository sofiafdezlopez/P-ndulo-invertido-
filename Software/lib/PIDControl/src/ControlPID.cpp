#include "Arduino.h"
#include "PIDControl.h"

PIDControl::PIDControl(double Kp, double Ki, double Kd, double dt,
                       double u0, PIDMode mode)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _dt(dt),
      _setpoint(0.0), _mode(mode),
      _output(u0), _limitOutput(false), _outMin(0.0), _outMax(0.0),
      _prevError(0.0), _integral(0.0),
      _d0(0.0), _d1(0.0), _fd0(0.0), _fd1(0.0)
{
    _error[0] = _error[1] = _error[2] = 0.0;

    // Coeficientes MODE_DISCRETE
    _A0 = Kp + Ki * dt + Kd / dt;
    _A1 = -Kp - 2.0 * Kd / dt;
    _A2 = Kd / dt;

    // Coeficientes MODE_FILTERED — parte PI
    _A0pi = Kp + Ki * dt;
    _A1pi = -Kp;

    // Coeficientes MODE_FILTERED — parte D
    _A0d = Kd / dt;
    _A1d = -2.0 * Kd / dt;
    _A2d = Kd / dt;

    // Filtro bilineal paso-bajo (N=5)
    const int N  = 5;
    double tau   = (Kp > 0.0) ? (Kd / (Kp * N)) : 1e-6;
    double alpha = dt / (2.0 * tau);
    _alpha1      = alpha / (alpha + 1.0);
    _alpha2      = (alpha - 1.0) / (alpha + 1.0);
}

void PIDControl::setOutputLimits(double min, double max) {
    if (min >= max) return;
    _limitOutput = true;
    _outMin = min;
    _outMax = max;
    _output = _clamp(_output);
}

void PIDControl::setSetpoint(double setpoint) {
    _setpoint = setpoint;
}

void PIDControl::reset() {
    _prevError = 0.0;
    _integral  = 0.0;
    _error[0] = _error[1] = _error[2] = 0.0;
    _d0 = _d1 = _fd0 = _fd1 = 0.0;
}

double PIDControl::compute(double measured_value) {
    switch (_mode) {

        case MODE_STANDARD: {
            double error      = _setpoint - measured_value;
            _integral        += error * _dt;
            double derivative = (error - _prevError) / _dt;
            _output           = _Kp * error + _Ki * _integral + _Kd * derivative;
            _prevError        = error;
            break;
        }

        case MODE_DISCRETE: {
            _error[2] = _error[1];
            _error[1] = _error[0];
            _error[0] = _setpoint - measured_value;
            _output   = _A0 * _error[0] + _A1 * _error[1] + _A2 * _error[2];
            break;
        }

        case MODE_FILTERED: {
            _error[2] = _error[1];
            _error[1] = _error[0];
            _error[0] = _setpoint - measured_value;

            // Parte PI incremental
            _output += _A0pi * _error[0] + _A1pi * _error[1];

            // Parte D con filtro bilineal
            _d1  = _d0;
            _d0  = _A0d * _error[0] + _A1d * _error[1] + _A2d * _error[2];
            _fd1 = _fd0;
            _fd0 = _alpha1 * (_d0 + _d1) - _alpha2 * _fd1;
            _output += _fd0;
            break;
        }
    }

    _output = _clamp(_output);
    return _output;
}

double PIDControl::_clamp(double value) {
    if (!_limitOutput) return value;
    if (value > _outMax) return _outMax;
    if (value < _outMin) return _outMin;
    return value;
}