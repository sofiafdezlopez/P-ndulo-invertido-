# Diseño Mecánico del Robot
En esta carpeta se incluyen los diseños y materiales mecánicos necesarios para la construcción de un robot autobalanceado.

La estructura de la carpeta es la siguiente:

- `README.md/`: Este fichero.
- `CHASIS.ipt/`: Modelado 3D del chasis en Inventor.
- `CHASIS_V2.stl/`: Archivo .stl del chasis en Inventor.
- `CHASIS_V2.gcode/`: Archivo .gcode del chasis en Inventor.
- `Componentes_mecanicos.md/`: Fichero que contiene la lista de componentes mecánicos.

## Estructura General
- **Dos ruedas motrices** (una de lado izquierdo y otra del lado derecho) con dos motores DC y reducturas, colocados respectivamente.
- **Chasis plano de dos plantas**:
    - **Inferior**: Bateria y motores.
    - **Superior**: Electrónica y sensor.

 ## Dimensiones Orientativas
- **Largo***~ 15 cm
- **Ancho**~ 10 cm
- **Altura**~ 3 cm
- **Distancia entre Ruedas**~ 15 cm
  
 ## Material Utilizado 
 - **Chasis**: PLA/PETG (Impresión 3D)
 - **Motores:** mini micromotores DC con engranajes de rueda dentada.
 - **Ruedas:** diámetro 6–7 cm, goma antideslizante (Mecanum).
 - **Batería:** Varta (9 V).
 - **Cables**
   
## Esquema simple (vista superior)
<pre>
┌─────────────────────────────────────────┐
│        Placa controladora arriba        │
│                                         │ 
[Rueda L]Motor               Motor[Rueda R]
│                                         │ 
│        Batería en el centro             │
└─────────────────────────────────────────┘
</pre>
