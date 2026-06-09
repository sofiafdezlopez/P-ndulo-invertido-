# Péndulo invertido

Somos dos alumnos del Grado de Robótica de la Universidad de Santiago de Compostela. En la asignatura de teoria de control se nos ha designado el trabajo de elaborar un robot "péndulo invertido".
El objetivo de este proyecto es que durante la duración de la materia realizaremos la construcción de robot y su control, que se basa en que el robot sea capaz de estabilizarse y mantenerse en una posición vertical.

Para ello hemos organizado todo nuestro trabajo de la siguiente manera:
- `Mecanica/`: Donde se encuentra toda la información sobre la parte mecánica de nuestro robot.
- `Electronica/`: Donde se encuentra toda la información sobre la parte electrica del robot.
- `Software/`: Donde se encuentra toda la información sobre la parte de la programación del robot.
- `README.md`: Fichero explicativo de todo el proyecto.
  
## Procedimiento a seguir

1. __Revisión de proyectos relacionados__ : Realizar una investigación de proyectos existentes para identificar robots similares ya implementados y documentados.
  
2. __Establecer objetivos__ : Debemos tener una idea clara de lo que tenemos que hacer.

3. __Configuración del hardware__ : Escoger los componentes necesarios para la elaboración del robot.

4. __Diseño del chasis__ : Diseñar el chasis ideal que cumpla con las especificaciones.

5. __Comprobación del funcionamiento de los componentes__ : Verificar el funcionamiento de todo el hardware antes del realizar el montaje.

6. __Montaje__ : Orden de montaje del robot.

7. __Implementación del software__ :

8. __Ajuste del PID__ :

## Revisión de proyectos relacionados

En cuanto a la busqueda de proyectos similares hemos encontrado videos donde te muestran el proceso de como construir y programar un robot autobalanceado pero en ningun momento lo documentan de manera que puedas hacerlo sin ver el video.

## Objetivos

- Elegir y adquirir componentes electromecánicos y electronicos.
- Diseñar e imprimir/fabricar el sistema mecánico del péndulo invertido mediante impresión 3D.
- Diseñar e implementar el controlador del péndulo invertido. Diseño de PCB, fabricación y
soldadura de componentes.
- Realizar la implementación del software de control en el microcontrolador ESP32.
- Sintonizar y ajustar el control PID para el péndulo invertido.
- Cumplir las especificaciones.

### Especificaciones

- El sistema debe ser capaz de __mantenerse en posición vertical__ con una tolerancia de ±5
- El sistema debe __compensar los empujones externos__ y volver a la posición vertical.
- El sistema debe ser capaz de poder __iniciar su funcionamiento desde cualquier posición__.

## Configuración del Hardware

Dado que el propósito principal del robot es medir el ángulo de inclinación respecto al suelo, la configuración de sus componentes es sencilla. Los elementos necesarios son los siguientes:

- ESP32
- 2 motores DC 
- 1 driver dual TB6612FNG
- Sensor MPU6050
- Bateria/pila 9V

## Diseño del chasis

A lo largo del proyecto, desarrollamos tres diseños del chasis, optimizandolo basándonos en los resultados de las pruebas:

- **Modelo 1 (Prototipo Inicial)**: Diseño sencillo de una sola planta con la PCB en la parte superior y la batería en la inferior. No resultó práctico ni eficiente, ya que no se tuvó en cuenta un espacio para el interruptor de encendido y este quedó colgando.

- **Modelo 2 (Estructura de Doble Planta)**: Rediseñamos el chasis para solucionar la falta de espacio del primer modelo. Elevamos la PCB para ubicar la batería justo debajo e incluimos una pequeña estructura para fijar el interruptor. Sin embargo, durante las pruebas, observamos que el robot funcionaba de manera más estable y eficiente a una menor altura respecto al suelo.

- **Modelo 3 (Diseño Final)**: Se mantiene la estructura del segundo modelo, pero introduce diferentes alturas en los enganches de los motores. Al elevar la posición de los motores, logramos bajar el cuerpo del robot, optimizando su centro de gravedad y mejorando su funcionamiento.

## Comprobación del funcionamiento de los componentes
Antes de realizar las pruebas de movimiento completo, es fundamental verificar de forma aislada que cada componente electrónico funciona correctamente. Seguimos esta guía de pruebas:

- **1. Test de Alimentación e InterruptorAcción**: Coloca la batería cargada y acciona el interruptor.

   - **Verificación**: Utiliza un polímetro (multímetro) para comprobar que el voltaje en los puntos clave de la PCB sea el correcto y confirma que el LED de estado del microcontrolador se enciende. Si el LED no se ilumina o detectas anomalías en el voltaje, desconecta la alimentación de inmediato para revisar el circuito en busca de cortocircuitos o polaridades invertidas antes de dañar los componentes.
     
- **2. Calibración y Lectura del Sensor de ÁnguloAcción**: Conecta el robot al ordenador mediante el cable USB y carga un script básico de prueba para el sensor.

   - **Verificación**: Abre el serial moinitor. Al mover el robot hacia adelante y hacia atrás, los valores del ángulo respecto al suelo deben variar de forma coherente.
     
- **3. Test de Giro de los MotoresAcción**: Eleva el robot (para que las ruedas no toquen el suelo) y ejecuta un código de prueba que haga girar los motores hacia adelante durante 2 segundos y hacia atrás otros 2 segundos.

   - **Verificación**: Asegúrate de que ambas ruedas giran en el sentido correcto. Si una rueda gira al revés, invierte la polaridad de sus cables en la PCB.
  
## Montaje

**Requisito previo**: La pieza o piezas que componen el chasis deben estar completamente impresas.Para este proyecto, la estructura se imprimió con una BAMBU LAB P1S de la universidad bajo las siguientes especificaciones:

- Material: PLA

- Configuración: Parámetros predeterminados por la universidad.

- Tiempo estimado de impresión: 2 horas y 20 minutos aprox.

Una vez retiradas las piezas de la base de impresión y limpios los soportes, se puede realizar el montaje.

**1. Materiales y Herramientas Necesarias**

Tornillos y tuercas: Especificadas en el apartado de mecánica.

Herramientas: Se necesita soldador de estaño, destornillador de estrella y bridas.

**2. Guía de Ensamblaje Paso a Paso**

Paso 1: El sistema de tracción: Se fijan los motores al chasis a la altura deseada.

Paso 2: Planta Inferior (Alimentación): Se coloca la pila/bateria en su enganche, implementado en el diseño del chasis.

Paso 3: Planta Superior (PCB): Se atornilla la PCB, en nuestro caso la enganchamos con bridas debido a que los agujeros de esta son muy pequeños, y se encaja el interruptor.

## Implementación del Software

## Ajuste del PID
