## Diseño del Software del Robot

En esta carpeta se incluye la implementación del diseño del software necesario para el autobalanceado del robot.

La estructura de la carpeta es la siguiente:
- `README.md/`: Este fichero.

La estructura de este fichero es la siguiente:

1. [Estructura](#estructura)
2. [Diagrama de bloques](#diagrama-de-bloques)
3. [Requisitos](#requisitos)
4. [Compilar y subir](#compilar-y-subir)
5. [Arquitectura](#arquitectura)


## Estructura
``` none
Software/
├── lib/
│   └── PIDControl/              ← Biblioteca de control PID propia
│       ├── src/
│       │   ├── PIDControl.h     ← Declaración de la clase y enumeración PIDMode
│       │   └── ControlPID.cpp   ← Implementación de los tres modos
│       ├── examples/            ← Sketches de ejemplo por modo
│       ├── library.properties   ← Metadatos para el IDE de Arduino
│       └── KEYWORDS.txt         ← Coloreado de sintaxis
├── src/
│   ├── main/
│   │   └── main.ino             ← Prototipo inicial (referencia histórica)
│   └── main_lib_pid/
│       └── main_lib_pid.ino     ← Programa principal con los tres modos PID
└── test/
    ├── calibracion_mpu6050/
    │   └── calibracion_mpu6050.ino  ← Utilidad de calibración del IMU
    ├── motores/
    │   └── motores.ino              ← Test de giro y dirección de motores
    └── mpu6050/
        └── mpu6050.ino              ← Test de lectura raw del sensor
```

## Diagrama de bloques
## Requisitos
## Compilación
## Arquitectura

