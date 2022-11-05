# Lab5Robotica: Control por Cinemática Inversa del Pincher PhantomX
## Universidad Nacional de Colombia
## Integrantes: Allan Giordy Serrato Gutierrez y Juan Fernando Ramirez Montes


## Objetivos

- Determinar el modelo cinemático inverso del robot Phantom X.
- Generar trayectorias de trabajo a partir del modelo cinemático inverso del robot.
- Implementar el modelo cinemático inverso del robot en MATLAB o Python.
- Operar el robot usando ROS a partir de trayectorias generadas en MATLAB o Python.

En primer lugar se tomaron las medidas de los eslabones con el fin de hallar los parámetros DH, permitiendo de esta manera establecer el modelo cinemático directo del problema. La referencia del robot es un Phantom X versión más reciente, en donde el primer eslabón cuenta con un desfase entre juntas.

## Video Práctica:


[![Video Practica 5](https://img.youtube.com/vi/AmfOuBePqY0/0.jpg)](https://www.youtube.com/watch?v=AmfOuBePqY0)

Video: https://www.youtube.com/watch?v=AmfOuBePqY0


### Resultados práctica: 

A continuación se muestra una foto del resultado final de todas las figuras dibujadas por el ppincher PhanthomX. 

![ResultadoZoom](https://user-images.githubusercontent.com/62154397/199907393-53f0a014-591c-4295-9c88-13eec24f23fb.jpg)


Poner aqui el video de la practica xd.

## Cinemática Directa

### Robot Phantom X Nuevo
![PincherNuevo](https://user-images.githubusercontent.com/51063831/199156872-a52c4ab6-e5cc-4d88-b0ec-e3d85763531a.jpeg)


### Robot Phantom X versión anterior
![PincherAntiguo](https://user-images.githubusercontent.com/51063831/199157145-2e5151ff-a131-4f75-98d7-2d8c1cfc9e31.jpeg)

Una vez tomadas las meidas de los eslabones, se llegó a la tabla con los parámetros DH que se muestran a continuación.

![DH](https://user-images.githubusercontent.com/51063831/199157443-94537c0b-56c5-4696-b476-4d6f08d8795b.PNG)

A partir de estos parámetros se creó una función con la capacidad de establecer una cinemática directa, función que se creo en el script de matlab con la intención de comprobar los resultados de la cinemática inversa realizada posteriormente. A continuación se aprecia el modelo graficado con el Toolboz de PeterCorke.

![Home](https://user-images.githubusercontent.com/62154397/199900168-41363e78-c2b4-4633-a1ff-2e5ff8a53cfe.jpg)

## Cinemática Inversa
Adicionalmente era necesario tener una función que se encargara de generar las posiciones a partir de unas coordenadas X Y Z asignadas por el usuario, por lo que la función de cinemática inversa fué hallada mediante el método geométrico. (EL método por matrices inversas tambien fue desarrollado en el script de MATLAb pero no fue implementado por su alta demanda computacional por punto). A continuación se muestran los calculos de los 4 ángulos correspondientes a cada servomotor del robot.

### q1:

EL primer ángulo por simple geométria es el arcotangente de las coordenadas (x,y) del TCP.

![theta1](https://user-images.githubusercontent.com/62154397/199897918-033210ed-1472-4735-a688-d1ac3c697166.jpg)

### q2 y q3: 
Para poder calcular estos ángulos, se hace uso del desacople de muñeca, tomando como muñeca la articulación q4. Lo que nos deja con un mecanismo 2R en 2D en el plano XZ. Debemos tener en cuenta que solo usaremos las soluciones de codo arriba para el módelo ya que el phantom estará limitado por la superficie limitada sobra la cual debe dbujar. Por ende nos queda el siguiente esquema, donde P4 es la posición de la articulación q4.

![theta3](https://user-images.githubusercontent.com/62154397/199898569-5ad8bdf7-cc60-451b-af21-fbaeb0c51f53.jpg)

Como podemos ver, en codo arriba nos queda un triangulo, que por ley de cosenos nos define q3 como: 

![image](https://user-images.githubusercontent.com/62154397/199899079-01dc5353-3874-46f4-82e2-7ded93a58c58.png)

Con q3 conocida, ahora podemos definir el siguiente triangulo para q2: 

![theta 2](https://user-images.githubusercontent.com/62154397/199899276-b651517d-c263-4599-aca3-7742e75969a6.jpg)

Se definen los parámetros geométricos del sistema.

![eq1theta2](https://user-images.githubusercontent.com/62154397/199899343-29cf0f25-01f3-4085-b4c5-f9391c27c6b1.jpg)

Y finalmente se define q2 como: 

![eq2theta2](https://user-images.githubusercontent.com/62154397/199899393-67bb4d8c-17fc-4a6e-9688-8a19dedc0a9a.jpg)

Para hallar q2 y 3 necesitamos saber los valores de las coordenadas de P4 en z y en x, por lo que definimos la posición de q4 como:

![eqq4](https://user-images.githubusercontent.com/62154397/199899782-370ab46d-e047-4e9f-bc1b-17556c887786.jpg)

Donde la matriz de transformación homogenea desde la herramienta (en este caso la punta del pinche que estará en contacto con el marcador) sera: 

![eq2q4](https://user-images.githubusercontent.com/62154397/199899868-c23f9aae-d8f6-4069-a962-4fa4b1458794.jpg)

y las coordenadas de q4 respecto al TCP será entonces: 

![eq1q4](https://user-images.githubusercontent.com/62154397/199900298-9307c3cf-04cd-45be-92bd-b79f7b36947c.jpg)

y con esos datos se pueden calcular los ángulos q2 y q3.

### q4: 

Para poder hallar q4, definimos un la orientación de muñeca, que finalmente será la orientación del eje x del marco de referencia del TCP respecto al eje x del marco de referencia del origen de la piza (x0), como se ve a continuación: 

![phi](https://user-images.githubusercontent.com/62154397/199901785-83ec7aad-56a9-499b-8ebb-9e8c9dfb2c4a.jpg)

Y por gemoetría definimos q4 como : 

![q4Sol](https://user-images.githubusercontent.com/62154397/199902183-5d58672f-507f-46bf-a4af-3fc28cc820e6.jpg)

En nuestro caso definimos phi como 0, para que el pincher pueda agarrar el marcador por los lados, de forma que el marcador quede totalmente vertical respecto a la superficie de dibujo. 

Hay que tener en cuenta que la última articulación del pincher, q5, define el grado de apertura de la pinza que agarra el marcador, así que su valor se tendrá que definir directamente, teniendo un módelo que recibe coordenadas (x,y,z) y un ángulo q5 para saber si cerrar o abrir el gripper del phantom. En la práctica se detecto que en q5=8.6° el pincher agarraba perfectamente el marcador, y que en q5=0° el gripper se abria totalmente.

Una vez se tiene esta base, se creó una función cuyo objetivo es trazar líneas, esta misma función se encarga de segmentar adecuadamente cada trazo con el fin de reducir el error asociado a la definición de posiciones propio de los robots con juntas rotacionales.

Una vez se tienen estos bloques base, fué posible iniciar con la definición del espacio de trabajo del robot, así como empezar con la organización del entorno disponible para poder realizar el trazado de todos los elementos solicitados en el laboratorio. A continuación se mostrará el análisis geométrico asociado a cada rutina, así como su implementación en el código con el fin de que el robot pudiese completar el trazado esperado.

## Código

### cinemática inversa

La función de cinemática inversa recibe entonces, las coordenadas (x,y,z) que se quieren alcanzar, la orientación phi de la muñeca en grados, y el ángulo en grados de la apertura del gripper (q5) que será de 0° para abierto y -8.6° para agarrar el marcador.

```console
# función de cinemática inversa
def Pos(x,y,z,phid,q5d): #(x,y,z) en cm, (phid,q5d) en grados.
    phi=rad(phid)
    q5=rad(q5d)

    #primer servo (q1):
    q1=np.arctan2(y,x) #rad
    
    #posición muñeca:
    a4=12.8
    PTCP4= np.array([[-a4], [0], [0],[1] ])
    #Matriz de transferencia de TCP respecto a 0:
    Rx90=np.array([[1,0,0], [0,np.cos(pi/2),-np.sin(pi/2)], [0,np.sin(pi/2),np.cos(pi/2)]])
    Rzphi=np.array([[np.cos(phi), -np.sin(phi), 0], [np.sin(phi), np.cos(phi), 0],[0, 0, 1]])
    R=mul_matrix(Rx90,Rzphi)
    H0TCP=np.array([[R[0,0],R[0,1],R[0,2],x], [R[1,0],R[1,1],R[1,2],y], [R[2,0],R[2,1],R[2,2],z], [0, 0, 0,1]])
    #pos muñeca respecto a base
    P04=mul_matrix(H0TCP,PTCP4)

    #Tercer servo (q3):
    a2=10.63 #cm
    a3=10.5 #cm
    cosq3=-(P04[0]**2+P04[2]**2-a2**2-a3**2)/(2*a2*a3)
    sinq3=-np.sqrt(1-cosq3**2)
    q3=np.arctan2(sinq3,cosq3) #rad

    #Segundo servo (q2):
    k1=a2+a3*cosq3
    k2=a3*sinq3
    q2=np.arctan2(P04[2],P04[0])+np.arctan2(k2,k1) #rad

    #Cuarto Servo (q4): 
    q4=phi-q2-q3 #rad.

    #Solución sin offsets.
    Solg=[q1,q2,q3,q4] #rad
    #Solución con los offsets.
    alpha2=np.arccos(3.39/10.63) #radians
    Solgof= [q1,-(q2+alpha2),-(q3-alpha2),-q4,q5] #rad
    return Solgof
```
### moverse a un punto:
```console
#mover al punto determinado.
def Mover(pos):
    now=asignar(Pos(pos))
    joint_publisher(now)
    return now
```

### generar una linea entre dos puntos: 
```console
#nueva función linea recta.
def linea(ini,fin,n):
  pasox = (fin[0]-ini[0])/n
  #print("El paso en X es", pasox)
  pasoy = (fin[1]-ini[1])/n
  #print("El paso en Y es", pasoy)
  pasoz = (fin[2]-ini[2])/n
  
  for i in range(n):
    #print("Punto: ", i)
    ini[0] = ini[0]+pasox
    ini[1] = ini[1]+pasoy
    ini[2] = ini[2]+pasoz
    Result=Pos(ini)
    joint_publisher(Result)
    
  print("Los puntos finales son: x ",ini[0],"y ",ini[1],"z ",ini[2]," ")
  print("Fin de la rutina")
```

### subir o bajar marcador en un punto determinado
``` console
def bajar(pos,ban,input):
    if input==0:#inversa
        now=asignar(Pos(pos))
    elif input==1:#directa
        now=asignar(pos)

    if ban==0:#subir
        now[1]+=rad(30)
        now[2]+=rad(10)
    elif ban==1:#bajar
        now[1]-=rad(12)
        now[2]-=rad(-10)

    joint_publisher(now)
```

### Publicador de postura al phantom: 
```console
# Función de publicación: (itera sobre la lista de posicipones)
def joint_publisher(postura):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False) 
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
    point = JointTrajectoryPoint()
    point.positions = postura
    point.time_from_start = rospy.Duration(0.5)
    state.points.append(point)
    pub.publish(state)
    print('published command')
    rospy.sleep(3)
```

### Rutinas implementadas

#### Tomar Marcador
```console
def marcador():
    #ir encima del marcador.
    Mover(posmarcador)
    #bajar hacia el marcador y cerrar griper.
    aux1=Mover(tomarmarcador)
    aux1[4]=closed
    joint_publisher(aux1) #cerrar gripper
    #subir marcador
    aux=asignar(tomarmarcador)
    aux[3]=closed
    bajar(aux,0,0)
    #ir a posicion de espera.
    Mover(puntoespera)
```

#### Arcos de limites inferior y superior
```console
# Arco Inferior y Arco Superior.
def arcos(lim):
    if lim==0: #arco inferior
        a=asignar(limiteinf)
    elif lim==1: #arco superior.
        a=asignar(limitesup)
    #Acercarse a punta izq.
    Mover(a)
    #bajar marcador.
    aux=asignar(a)
    aux[2]-=5
    linea(a,aux,1)
    #bajar(a,1)

    Approach=Pos(a) #pose.

    #Dibujar Arco.
    q10=rad(-45) #q1 inicial.
    aux1=asignar(Approach) 
    aux1[0]=q10+rad(90)
    joint_publisher(aux1)

    #subir marcador
    aux2=asignar(aux1)
    aux2[0]=q10+rad(90)
    bajar(aux2,0,1)

    #ir a punto de espera:
    Mover(puntoespera)
```

#### Triangulo equilatero
```console
#Triangulo Equilatero.  
def Newtriangulo():
    #xt=20
    #yt=-1
    punto1 = [20,-1,zoff-0.5,closed]
    Mover(punto1)
    punto2 = [20,3,zoff-0.5,closed]
    punto3 = [23.46,1,zoff,closed]
    punto4 = [20,-1,zoff,closed]
    linea(punto1,punto2,8)
    linea(punto2,punto3,8)
    linea(punto3,punto4,8)
    
    #levantar marcador:
    bajar(punto4,0,0)

    #ir punto de espera:
    Mover(puntoespera)
 ```
 
 #### Puntos equidistantes
 ```console
 #punto equidistantes.
def puntos():
    #moverse al origen
    ini=asignar(puntosorg)
    ini[2]+=2
    Mover(ini)

    aux=asignar(puntosorg)
    #hacer puntos
    for i in range(5):
        bajar(aux,1,0)
        bajar(aux,0,0)
        aux[0]+=2
        aux[2]+=0.22
        if i==4:
            aux[2]+=1
        Mover(aux)

    #a punto de espera.
    Mover(puntoespera)
```
#### 3 lineas paralelas
```console
def lineas(n):#subirlo en z.
    #paralelasorg=[18.3,-8,zoff-1,closed]
    #mover a punto y bajarmarcador
    auxl=asignar(paralelasorg)
    auxl[0]-=0.5
    auxl[1]+=0.5
    auxl[2]+=2.6
    Mover(auxl) #15,-5
    bajar(auxl,1,0)

    #linea1
    punto1=[18.3,-4,zoff-1,closed]
    linea(paralelasorg,punto1,n)
    bajar(punto1,0,0)#subir marcador.

    #linea 2
    punto2=[20,-8,zoff-1,closed]
    punto3=[20,-4,zoff-1,closed]
    #acercarse a punto 2 y bajar
    auxl=asignar(punto2)
    auxl[0]-=1
    auxl[2]+=4
    Mover(auxl)
    bajar(auxl,1,0)
    #hacer linea
    linea(punto2,punto3,n)
    bajar(punto3,0,0)#subir

    #linea 3
    punto4=[21.7,-8,zoff-0.5,closed]
    punto5=[21.7,-4,zoff-0.5,closed]
    #acercarse y bajar marcador
    auxl=asignar(punto4)
    auxl[0]-=1
    auxl[2]+=4.2
    Mover(auxl)
    bajar(auxl,1,0)
    #dibujar linea
    linea(punto4,punto5,n)
    #alzar marcador
    bajar(punto5,0,0)#subir

    #ir a punto de espera.
    Mover(puntoespera)
```

#### Trayectoria curva

Debido a los problemas por el desplazamiento en z del phantom al subir por el eje x, se determino hacer un medio circulo de 0° a 180° con una función que suma z constantemente, y que hace otro medio circulo de 180° a 360° con una z constante.

```console
#trayectoria curva
def curva(a,b,r,z,q5d,thetamaxd,sup):
    #centro (a,b) en #cm
    # r en cm.
    off=0
    centro=[a,b,z-0.5,q5d]
    #ir al centro
    cir=[centro]

    thetamax=thetamaxd
    
    q5=rad(q5d) #apertura del gripper
    if thetamax > 200:
        thetamax%=200
    thetamax=rad(thetamaxd)
    
    n=int(ncurv*thetamaxd/180) #No. de puntos.
    print(n)
    dth=(thetamax)/n #paso de angulo.
    #lista de angulos.
    if sup==0:#inf
        theta=[pi/2]
    if sup==1:#sup
        theta=[-pi/2]
    
    for i in range(1,n):
        theta.append(theta[i-1]+(dth))
    #print(theta)
    #calculo de x y y.
    x=[]
    y=[]
    zq=[]
    k=0.2
    pasoi=[0,1,2,3,4,5]
    #dz para cada pi/4
    for i in range(n):
        if sup==1: #arco sup
            paso=pasoi[i%4]*5/4
            if theta[i]< (thetamax+pi/2): #n/2
                zq.append(z+0.6+paso*k)
            elif theta[i] < (thetamax+rad(220)): #n
                zq.append(z+0.6-paso*k)
        elif sup==0: #arco inf.
            zq.append(z+0.3)

    print(zq)

    for i in range(0,n):
        x.append(r*(np.cos(theta[i]))+a)
        y.append(r*(np.sin(theta[i]))+b)
    
    #print(x)
    #print(y)
    #ir a pto inicio circulo (mitad izq)
    #Mover(centro) #movel al centro.
    #bajar(centro,0,0) #subir marcador.
    inicio=asignar(centro) #punto inicial.
    inicio[0]=x[0]
    inicio[1]=y[0]
    inicio[2]+=0 #dz entre circulo y centro.
    #cir.append(aux1)
    #Mover(inicio)#mover a inicio 
    #bajar(inicio,0,0)#subir marcador

    #Crear array con puntos de circulo
    for i in range(0,n):
        result2=[x[i], y[i], zq[i], q5]
        cir.append(result2)

    #Dibujar circulo.
    for i in range(len(cir)-1):
        linea(cir[i],cir[i+1],1)
        #Mover(cir[i+1])
    
    #subir marcador:
    #bajar(cir[-1],0,0)
    #ir a punto final: 
    final=asignar(centro)
    final[0]=x[n-1]
    final[1]=y[n-1]
    final[2]=z
    #print(final)
    #Mover(final)
    bajar(final,0,0)#subir marcador
```    

#### Circulo 
``` console
#Circunferencia
def circulo(a,b,r):
    #dibujar circulo
    curva(a,b,r,zoff,closed,200,1)#superior.
    curva(a,b,r,zoff,closed,200,0)#inferior
    #ir a punto de espera.
    Mover(puntoespera)
``` 

#### Iniciales FA
``` console
#iniciales
def iniciales():
    posA=[10,15,zoff-0.2,closed]
    #la A:
    #aproximacion
    #Mover(posiniciales)
    auxi=asignar(posA)
    auxi[2]-=0.7
    Mover(auxi)
    #bajar(posA,1,0)
    #linea 1
    punto1=[15,13,zoff,closed]
    #Mover(punto1)
    linea(posA,punto1,4)
    #linea 2
    punto2=[10,11,zoff-0.3,closed]
    linea(punto1,punto2,4)
    bajar(punto2,0,0)
    #linea 3
    punto3=[12,14.5,zoff-0.2,closed]
    Mover(punto3)
    punto4=[12,11.3,zoff-0.2,closed]
    linea(punto3,punto4,3)
    bajar(punto4,0,0)#subir"""

    #la F:
    #aproximacion
    xf=18
    yf=13
    posF=[xf,yf,zoff-0.2,closed]
    auxi=asignar(posF)
    auxi[2]-=0.5
    Mover(auxi)
    #linea 1:
    punto1=[xf+5,yf,zoff,closed]
    linea(posF,punto1,7)
    #linea 2:
    punto2=[xf+5,yf-3,zoff,closed]
    linea(punto1,punto2,4)
    bajar(punto2,0,0)#subir
    #linea 3:
    punto3=[xf+2.5,yf,zoff,closed]
    Mover(punto3)
    punto4=[xf+2.5,yf-3,zoff,closed]
    linea(punto3,punto4,4)
    bajar(punto4,0,0) #subir

    #ir a espera
    Mover(puntoespera)
``` 

#### Figura Libre (Paraguas)
``` console
#figura libre
def libre():
    #xl=22
    #yl=-2
    posLibre=[22-0.6,-2,zoff-0.5,closed]
    Mover(posLibre)
    #cruz
    #linea1:
    pos1=[22+4,-2,zoff-0.5,closed]
    linea(posLibre,pos1,6)
    bajar(pos1,0,0) #subir
    #linea2:
    pos2=[22+2,-2-2,zoff,closed]
    pos3=[22+2,-2+2,zoff,closed]
    Mover(pos2)
    linea(pos2,pos3,6)
    bajar(pos3,0,0)#subir

    #curva superior
    poscurva=[22+2,-2-2,zoff,closed]
    Mover(poscurva)
    #dibujar semicirculo
    curva(24,-2,2,zoff,closed,190,1)

    #posicion de espera:
    Mover(puntoespera)
```

#### Descargar marcador
```console
# Descargar marcador.
def descargue():
    #ir encima del marcador.
    aux1=asignar(posmarcador)
    aux1[3]=closed
    Mover(aux1)
    #bajar hacia el marcador y abrir griper.
    aux2=Mover(tomarmarcador)
    aux2[4]=open
    joint_publisher(aux2) #abrir gripper
    #subir marcador
    aux=asignar(tomarmarcador)
    aux[3]=open
    bajar(aux,0,0)
    #ir a home.
    Mover(homex)
 ```
 
 
### Interfaz de usuario

Se generó un ainterfaz en la al oprimir una de las diferentes letras listadas en el código a continuación, se generaba una rutina distinta y por ende un nuevo dibujo. Se crearon los inputs para moverlos a un puntos de espera, a home y a un punto de prueba para ayudar con los multiples ensayos con el robot, donde test se usaba para confimar el envio correcto de datos al phantom.

```console
# FUNCIÓN DE ACCESO:
def interfaz():

    while not rospy.is_shutdown():
        #control de mov. con teclas.
        key=input()
        if key == 'q':
            #marcador
            marcador()
            key = ' '
        elif key == 'w':
            #arco inf
            arcos(0)
            key = ' '
        elif key == 's':
            #arco sup.
            arcos(1)
            key = ' '
        elif key == 'e':
            #iniciales FA
            iniciales()
            key = ' '
        elif key == 'a':
            #Trianglulo equilatero.
            Newtriangulo()
            key = ' '
        elif key == 'd':
            #Circunferencia.
            circulo(centroCir[0],centroCir[1],radcir)
            key = ' '
        elif key =='x':
            #3 lineas paralelas
            lineas(5)
            key== ' '
        elif key =='z':
            #descargar marcador
            descargue()
            key== ' '
        elif key=='m':
            Mover(puntoespera)
            key= ' '
        elif key=='t':
            Mover(test)
            key=' '
        elif key=='h':
            Mover(homex)
            key=' '
        elif key=='p':
            #puntos equidistantes
            puntos()
        elif key=='f':
            key=' '
            libre()
```

### Bucle infinito

Por medio de un bucle infinito que llama a la función interfaz(), se crea una interfaz de usuario que siempre le esta pidiendo el input de teclado al usuario y llama las rutinas a realizar según dicho input.

```console
if __name__ == '__main__':
    try:
        interfaz()
    except rospy.ROSInterruptException:
        pass
```

## Comparación práctica y modelado: 


## Entregables: 

- Codigo. (.py, ROS y .yaml)
- Modelos, fotografias y descripción de la base portaherramienta y el marcador.
- Video de la práctica. (mostrando los scripts en vivo)
- Descripción de la solución planteada (teoría).


