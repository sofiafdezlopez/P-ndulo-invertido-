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
    // Kp: ganancia proporcional
    // Ki: ganancia integral
    // Kd: ganancia derivativa
    // dt: período de muestreo en segundos
    // u0: valor inicial de salida (por defecto 0)
    // mode: modo de operación (STANDARD, DISCRETE o FILTERED)
    PIDControl(double Kp, double Ki, double Kd, double dt,
               double u0 = 0.0, PIDMode mode = MODE_STANDARD);

    // Establece los límites mínimo y máximo para la salida del controlador
    // Útil para proteger actuadores (ej: limitar PWM entre -170 y 170)
    void   establecerLimiteSalida(double min, double max);
    
    // Establece el valor objetivo (setpoint) que el controlador intenta alcanzar
    void   establecerSetpoint(double setpoint);
    
    // Calcula la salida del controlador basado en la lectura actual del sensor
    // Retorna la acción de control a aplicar al motor
    double calcular(double valor_medido);
    
    // Reinicia el estado interno del controlador (integral, buffers, etc.)
    // Útil al cambiar de modo o después de una caída
    void   reiniciar();

    // Retorna la salida actual del controlador
    double obtenerSalida()   const { return _output; }
    
    // Retorna el valor objetivo actual
    double obtenerSetpoint() const { return _setpoint; }

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
};

#endif