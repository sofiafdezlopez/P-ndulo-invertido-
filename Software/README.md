## Diseño del Software del Robot

En esta carpeta se incluye la implementación del diseño del software necesario para el autobalanceado del robot.

La estructura de la carpeta es la siguiente:
- `README.md/`: Este fichero.

La estructura de este fichero es la siguiente:

1. [Estructura](#estructura)
2. [Diagrama de bloques](#diagrama-de-bloques)
3. [Compilación](#compilación)
4. [Arquitectura](#arquitectura)


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
``` none
┌─────────────────────────────────────────────────────────┐
│                        ESP32                            │
│                                                         │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ MPU6050  │───▶│   Filtro     │───▶│  Controlador │  │
│  │ IMU I²C  │    │complementario│    │  PID (modo    │  │
│  └──────────┘    └──────────────┘    │  seleccionado)│  │
│                       α=0,98         └──────┬────────┘  │
│                                             │           │
│  ┌──────────────────────────┐               ▼           │
│  │   Serial (115200 baud)   │    ┌──────────────────┐   │
│  │  S → MODE_STANDARD       │    │  Driver TB6612   │   │
│  │  D → MODE_DISCRETE       │    │  Motor A  Motor B│   │
│  │  F → MODE_FILTERED       │    └──────────────────┘   │
│  └──────────────────────────┘                           │
└─────────────────────────────────────────────────────────┘
```


## Compilación

Instalar la biblioteca PIDControl: en el Arduino IDE, Sketch → Include Library → Add .ZIP Library... y seleccionar la carpeta `lib/PIDControl/`.

1- Abrir `src/main_lib_pid/main_lib_pid.ino`.

2- Seleccionar la placa ESP32 Dev Module y el puerto correspondiente.

3- Compilar y subir.

4- Abrir el Monitor Serie a 115200 baud para ver telemetría y enviar comandos.

Para calibrar el MPU6050 (solo necesario si se cambia el sensor o el montaje): abrir y ejecutar `test/calibracion_mpu6050/calibracion_mpu6050.ino` y copiar los offsets resultantes en el sketch principal.

## Arquitectura

**Estimación del ángulo — Filtro complementario**

- El ángulo de inclinación se estima fusionando dos fuentes del MPU6050:
    
   * Acelerómetro: proporciona el ángulo absoluto mediante atan2, pero es ruidoso ante vibraciones.
   * Giroscopio: integra la velocidad angular para obtener el ángulo incremental; preciso a corto plazo pero deriva con el tiempo.
    
- El filtro complementario combina ambas fuentes:
```none
ang_y = α · (ang_y_prev + gy_dps · dt) + (1 − α) · accel_ang
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Con α = 0,98: el 98 % del peso recae en el giroscopio (respuesta rápida) y el 2 % en el acelerómetro (corrección de deriva).


**Biblioteca PIDControl — Tres modos de control**

- La clase PIDControl implementa tres variantes del controlador PID discreto:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`MODE_STANDARD — PID clásico posicional`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Calcula la salida acumulando la integral del error y la derivada por diferencias finitas:
```none
u(k) = Kp·e(k) + Ki·Σe·dt + Kd·(e(k) − e(k−1))/dt
```
<br>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`MODE_DISCRETE — Forma recurrente directa`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Expresa la salida directamente en función de los tres últimos errores, sin estado acumulado:
```none
u(k) = A0·e(k) + A1·e(k−1) + A2·e(k−2)
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;donde 
```none
A0 = Kp + Ki·dt + Kd/dt, A1 = −Kp − 2·Kd/dt, A2 = Kd/dt.
```

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`MODE_FILTERED — PI incremental con derivada filtrada`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Suma incrementalmente la parte PI y aplica un filtro paso-bajo bilineal sobre la parte derivativa para atenuar el ruido:
```none
Δu_PI(k) = (Kp + Ki·dt)·e(k) − Kp·e(k−1)
d(k)      = A0d·e(k) + A1d·e(k−1) + A2d·e(k−2)
fd(k)     = α1·(d(k) + d(k−1)) − α2·fd(k−1)    ← filtro bilineal
u(k)      = u(k−1) + Δu_PI(k) + fd(k)
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;La constante de tiempo del filtro es τ = Kd/(Kp·N), con N = 5 por defecto.


**Control de motores**

- La salida del controlador PID se aplica a ambos motores en sentido opuesto (el robot avanza para compensar la inclinación). La función moveMotor gestiona la dirección mediante los pines IN1/IN2 del driver y la velocidad mediante PWM:

    * Salida positiva → motor A avanza, motor B retrocede.
    * Salida negativa → motor A retrocede, motor B avanza.

- PWM limitado a 170 cuentas (≈ 6,7 V efectivos sobre 9 V de batería).


**Protección por caída**

- Si el ángulo de inclinación supera ±35°, el sistema detiene los motores, reinicia todos los controladores PID y espera a que el robot sea recolocado manualmente. Esto evita que los motores fuercen contra el suelo y protege el hardware.


**Cambio de modo en tiempo de ejecución**

- Enviando S, D o F por el Monitor Serie se cambia el controlador activo sin necesidad de reiniciar el robot ni recompilar. Al cambiar de modo se reinician todos los controladores para evitar transitorios causados por valores residuales en los buffers de error.
