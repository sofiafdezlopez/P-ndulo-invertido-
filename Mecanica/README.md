# Diseño Mecánico del Robot
En esta carpeta se incluyen los diseños y materiales mecánicos necesarios para la construcción de un robot autobalanceado.

La estructura de la carpeta es la siguiente:

- `README.md/`: Este fichero.
- `Diseño3D.zip/`: Carpeta comprimida del modelado 3D del robot en Inventor.
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
