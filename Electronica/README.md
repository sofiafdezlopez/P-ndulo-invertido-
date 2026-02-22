# Diseño Electrónico del Robot
En esta carpeta se incluyen los diseños y materiales electrónicos necesarios para la construcción de un robot autobalanceado.

La estructura de la carpeta es la siguiente:

- `README.md/`: Este fichero.
- `PCB.zip/`: Carpeta comprimida del modelado de la PCB en Kicad.
- `Componente.md/`: Fichero que contiene la lista de componentes electrónicos.

## Objetivo del Diseño
Placa compacta que aloja:
- Un **ESP32** como MCU/Wi-Fi/Bluetooth.
- Control de **4 motores DC** (2 drivers duales TB6612FNG).
- Lectura de sensores IR y MPU6050.
- Gestión de **alimentación** desde LiPo.
- Conectores para batería, motores, sensores y programación.

## Especificaciones PCB
- **Dimensiones:** ~ 10 × 10 cm 
- **Capas:** 2.

## Adaptaciones de la PCB
La PCB original fue diseñada para su uso en un robot de tipo sumo. Sin embargo, en este proyecto no se utilizarán algunos de los elementos incluidos en el diseño, como por ejemplo los conectores para sensores infrarrojos y finales de carrera, ya que no son necesarios para el funcionamiento del sistema implementado.

Por este motivo, antes del montaje se procederá a eliminar las secciones de la PCB que no serán utilizadas, recortando las partes innecesarias y dejando únicamente la zona correspondiente a los componentes funcionales del proyecto. Esta modificación permite simplificar la estructura del hardware y reducir el tamaño final del módulo electrónico.

## Esquema funcional
- **Entrada batería** → protección (diodo) → **regulador a 5V** → 5V 
- 5V  → ESP32 → **regulador a 3.3V** → 3.3V
- 3.3V: pines GPIO → drivers / sensores / I²C
- **Driver motores (TB6612)**: alimentado a 12V (Vm), señales IN1/IN2 desde ESP32, pin PWM para velocidad.
- Sensores: Conectados a GPIOs del ESP32.

## Protección y seguridad
- Fusible rearmable (PTC) en la entrada de la batería.
- Diodo de protección contra conexión inversa (schottky).
- Indicador LED de encendido (5V) y LED de estado ESP32.

## Prueba de la resistencia de protección en TX0

El uso del GPIO correspondiente a TX0 puede ocasionar problemas. Para mitigar el riesgo de quemar el pin GPIO y las interrupciones en la comunicación serial, así como para facilitar el debug y los tests de forma cómoda y segura, aplicamos las siguientes medidas:

- Colocar una **resistencia en serie** de 4,7KΩ entre el Pin y el Bumper (protección ante el cortocircuito).
- Usa el GPIO TX0 para un bumper (mitigar entradas).

### Procedimiento de prueba

Realiza este procedimiento con el robot APAGADO y desconectado.

1. Pon el multímetro en modo Resistencia (Ω).
2. Coloca una punta en el pin GPIO 1 del ESP32 (o en la soldadura correspondiente).
3. Coloca la otra punta en GND (cualquier punto de masa).
4. **Presiona el Bumper**.
5. **Verifica la lectura:** Debes ver el valor aproximado de tu resistencia (**4.7 kΩ**).

- Si ves **0 Ω** (o muy cerca de cero): La resistencia no está en serie, hay un cortocircuito directo. **No lo enciendas hasta solucionarlo**.
- Si OL (Over Limit): El interruptor no está cerrando el circuito.

### ¿Por qué funciona esto?

Esta configuración es segura y funcional por dos razones matemáticas:

1. **Seguridad (Protección del Pin):**
Si el pin TX0 intenta enviar datos (3.3V) mientras pulsas el botón, la resistencia limita la corriente máxima, evitando quemar el transistor interno del ESP32.

$$I_{max} = \frac{3.3V}{4700\Omega} \approx 0.7 mA$$

*(El ESP32 soporta hasta 12mA, por lo que 0.7mA es totalmente seguro).*

2. **Funcionalidad (Lectura correcta):**
Al leer el botón, el ESP32 activa una resistencia interna de pull-up (~45kΩ). Al pulsar, se crea un divisor de tensión:

$$V_{pin} = 3.3V \times \frac{4.7k\Omega}{4.7k\Omega + 45k\Omega} \approx 0.31V$$

*(0.31V es interpretado como un "LOW" lógico por el ESP32, por lo que detecta la pulsación correctamente).*

Entoces:

- Sin presionar: El pin está en HIGH (3.3V) gracias al pull-up interno. digitalRead devuelve 1.
- Presionado: El pin baja a 0.31V (LOW). digitalRead devuelve 0.

