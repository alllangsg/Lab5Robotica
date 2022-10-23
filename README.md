# Lab5Robotica
 Control por Cinemática Inversa del Pincher PhantomX

## Puntos a Realizar:

- Espacio de trabajo del PhantomX.
- Módelo de cinemática inversa.
- Implementación de trayectorias.
- COnstruir un portaherramienta para el marcador.
- Práctica de escritura.
- Script de Python+ROS.

## Práctica: 

### Rutinas: 
- Cargar Herramienta: Sujetar y Levantar el marcador.
- Espacio de Trabajo: Arco inferior y arco Superior.
- Dibujo de iniciales: JA.
- Dibujar un triángulo equilatero, una circunferencia, 3 lineas rectas paralelas.
- Dibujo libre con 3 trazos rectos y curvos.
- Descargar la herramienta: Soltar el marcador en el portaherramienta.

Todas las Rutinas deben terminar en una posición de espera.
Las demostraciones deben iniciar con el brazo tomando el marcador y finalizar con el brazo descargando el marcador.

### UI:

- El usuario debe seleccionar la rutina requerida con input de teclado.
- No se debe poder ejecutar la rutina si no se sujeta el marcador.
- Mensajes en la consola: 
    - Estado de la herramienta. 
    - Posición actual del brazo.
    - Rutina seleccionada.
    - Tiempo de ejecución de la última rutina. 

### Verificación Dimensional:

Verificar las dimensiones, calidad del brazo, radio y homogeneidad de todos los trazos.
Escoger e implementar una metodología para medir la precisión del brazo.
Tomar fotos de los trazos y compararlos con imagenes de las trayectorias ideales.
Analizar la repitibilidad de los trazos haciendo multiples rutinas desde el cargue hasta el descargue del marcador.

## Entregables: 

- Codigo. (.py, ROS y .yaml)
- Modelos, fotografias y descripción de la base portaherramienta y el marcador.
- Video de la práctica. (mostrando los scripts en vivo)
- Descripción de la solución planteada (teoría).
