# Lab5Robotica: Control por Cinemática Inversa del Pincher PhantomX
## Universidad Nacional de Colombia
## Integrantes: Allan Giordy Serrato Gutierrez y Juan Fernando Ramirez Montes


## Objetivos

- Determinar el modelo cinemático inverso del robot Phantom X.
- Generar trayectorias de trabajo a partir del modelo cinemático inverso del robot.
- Implementar el modelo cinemático inverso del robot en MATLAB o Python.
- Operar el robot usando ROS a partir de trayectorias generadas en MATLAB o Python.

En primer lugar se tomaron las medidas de los eslabones con el fin de hallar los parámetros DH, permitiendo de esta manera establecer el modelo cinemático directo del problema. La referencia del robot es un Phantom X versión más reciente, en donde el primer eslabón cuenta con un desfase entre juntas.


### Robot Phantom X Nuevo
![PincherNuevo](https://user-images.githubusercontent.com/51063831/199156872-a52c4ab6-e5cc-4d88-b0ec-e3d85763531a.jpeg)


### Robot Phantom X versión anterior
![PincherAntiguo](https://user-images.githubusercontent.com/51063831/199157145-2e5151ff-a131-4f75-98d7-2d8c1cfc9e31.jpeg)

Una vez tomadas las meidas de los eslabones, se llegó a la tabla con los parámetros DH que se muestran a continuación.

![DH](https://user-images.githubusercontent.com/51063831/199157443-94537c0b-56c5-4696-b476-4d6f08d8795b.PNG)

A partir de estos parámetros se creó una función con la capacidad de establecer una cinemática directa, en caso de ser requerida para ajustar algún parámetro de configuración durante el desarrollo del laboratorio, el código asociado a esta función se muestra a continuación:

Adicionalmente era necesario tener una función que se encargara de generar las posiciones a partir de unas coordenadas X Y Z asignadas por el usuario, por lo que la función de cinemática inversa fué programada:

Una vez se tiene esta base, se creó una función cuyo objetivo es trazar líneas, esta misma función se encarga de segmentar adecuadamente cada trazo con el fin de reducir el error asociado a la definición de posiciones propio de los robots con juntas rotacionales, la función quedó de la siguiente manera:

Una vez se tienen estos bloques base, fué posible iniciar con la definición del espacio de trabajo del robot, así como empezar con la organización del entorno disponible para poder realizar el trazado de todos los elementos solicitados en el laboratorio. A continuación se mostrará el análisis geométrico asociado a cada rutina, así como su implementación en el código con el fin de que el robot pudiese completar el trazado esperado.

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


#Video completo laboratorio

