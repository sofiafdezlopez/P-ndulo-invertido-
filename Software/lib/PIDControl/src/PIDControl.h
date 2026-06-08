#ifndef PIDControl_h
#define PIDControl_h

#include "Arduino.h"

enum PIDMode {
    MODE_STANDARD = 0,  // PID clásico posicional
    MODE_DISCRETE,      // PID discreto (forma de velocidad)
    MODE_FILTERED       // PID con derivada filtrada
};

class PIDControl {
public:
    PIDControl(double Kp, double Ki, double Kd, double dt,
               double u0 = 0.0, PIDMode mode = MODE_STANDARD);

    void   setOutputLimits(double min, double max);
    void   setSetpoint(double setpoint);
    double compute(double measured_value);
    void   reset();

    double getOutput()   const { return _output; }
    double getSetpoint() const { return _setpoint; }

private:
    double _Kp, _Ki, _Kd, _dt;
    double _setpoint;
    PIDMode _mode;
    double _output;
    bool   _limitOutput;
    double _outMin, _outMax;

    // Estado MODE_STANDARD
    double _prevError;
    double _integral;

    // Buffer de errores para MODE_DISCRETE y MODE_FILTERED
    double _error[3];

    // Coeficientes MODE_DISCRETE
    double _A0, _A1, _A2;

    // Coeficientes MODE_FILTERED
    double _A0pi, _A1pi;
    double _A0d, _A1d, _A2d;
    double _alpha1, _alpha2;
    double _d0, _d1, _fd0, _fd1;

    double _clamp(double value);
};

#endif