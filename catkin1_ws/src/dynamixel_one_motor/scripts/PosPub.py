'Cinematica Inversa PhantomX'

from fileinput import close
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#constantes de interes.
pi=np.pi
open=15
closed=-8.6 #angulo para gripper cerrado. °
zoff=9 #altura marcador en cm.
ng=7 #numero puntos por trayectoria.
ncurv=15 #numero de puntos para curva 180°
radcir= 2 #radio circulo en cm.
lonlin= 5 #longitud lineas en cm.
alturaF= 4 #Altura de la F en cm. q
anchoF = 4 #ancho F en cm
ladotrian= 5 #lados del triangulo.
short=6 #distancia para bajar/ #puntos tray corta.
shorter=3

# pasar a radianes.
def rad(degrees):
    angle=degrees*np.pi/180
    return angle

#asignar un array a uno
def asignar(current):
  new=[]
  for i in range(len(current)):
    new.append(current[i])
  return new

#graficar.
def graficar(puntos):
    x=[]
    y=[]
    for i in range(len(puntos)):
        x.append(puntos[i][0])
        y.append(puntos[i][1])

    plt.figure(1)
    plt.plot(x, y)
    plt.show()

#unir dos arrays
def unir(target,arr2):
    aux=asignar(target)
    for i in range(len(arr2)):
        aux.append(arr2[i])
    return aux

#posiciones cartesianas de interes:
limiteinf=[10,-9,zoff+3.4,closed]
limitesup=[21,-20,zoff+5,closed]
homex=[25,0,20,open]
puntoespera=[20,5,zoff+short,closed]

posmarcador=[14.4,-23.5,zoff+7.5,open]
tomarmarcador=[14.5,-23.5,zoff+1.4,open]

triangorg=[20,0,zoff,closed]
centroCir=[20,6]
paralelasorg=[18.3,-8,zoff-1,closed]
posiniciales=[10,15,zoff,closed]
test=[25,5,zoff+3,closed]
libreorg=[10,10,zoff,closed]
puntosorg=[12,-10,zoff+0.7,closed]

# posturas de interes:
home=[0,0,0,0,0]
homec=[0,0,0,0,rad(closed)]
espera=[10,-10,10,0,rad(closed)]


def Pos(array):
    #pasar a mm.
    x=array[0]*10
    y=array[1]*10
    z=array[2]*10
    q5=rad(array[3])
    #Triangulo constante
    hipo=106.5; #distancia diagonal entre q2 y q3
    cate=np.sqrt(hipo**2-100**2) #cateto de la diagonal
    angle = np.arctan2(cate,100)
    
    q1=np.arctan2(y,x)
    
    dmar0 = np.array([[x], [y], [z]])
    d3 = np.multiply(np.array([[np.cos(q1)], [np.sin(q1)], [0]]),100)
    dw0 = dmar0-d3
    
    dz = dw0[2]-94
    r2 = (dw0[0])**2+(dw0[1])**2+(dz)**2
    r = np.sqrt(r2)
    
    #Teorema del coseno
    alpha = np.arccos((100**2+r2-hipo**2)/(2*r*100))
    beta = np.arccos((r2+hipo**2-100**2)/(2*r*hipo))
    gamma = np.arccos((hipo**2+100**2-r2)/(2*hipo*100))

    theta = np.arctan2(dz,np.sqrt((dw0[0])**2+(dw0[1])**2))
    psi = np.pi-alpha
    phi = np.pi-psi-theta
    omega = np.pi-phi
    kappa = omega-np.pi/2
    
    q2= np.round(np.pi/2-theta-beta-angle-np.pi/18,3)
    q3= np.round(np.pi-gamma-(np.pi/2-angle),3)
    q4 =np.round(-(np.pi/2-kappa),3)
    Result=[float(np.round(q1,3)),float(-q2),float(-q3),float(-q4),float(q5)]

    return Result
    
#mover al punto determinado.
def Mover(pos):
    now=asignar(Pos(pos))
    joint_publisher(now)
    return now

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

#nueva función linea recta.
def linea(ini,fin,n):
  pasox = (fin[0]-ini[0])/n
  #print("El paso en X es", pasox)
  pasoy = (fin[1]-ini[1])/n
  #print("El paso en Y es", pasoy)
  pasoz = (fin[2]-ini[2])/n
  #print("El paso en Z es", pasoz)
  #print("Ahora vamos a enviar cada punto de la línea")
  #print("Puntos originales:",ini[0]," ",ini[1]," ",ini[2]," ")
  #print("Puntos objetivo:",fin[0]," ",fin[1]," ",fin[2]," ")
  #primer punto
  for i in range(n):
    #print("Punto: ", i)
    ini[0] = ini[0]+pasox
    ini[1] = ini[1]+pasoy
    ini[2] = ini[2]+pasoz
    Result=Pos(ini)
    joint_publisher(Result)
    #print("Nuevos puntos en trayectoria",ini[0]," ",ini[1]," ",ini[2]," ")
    #print("Repite ciclo")
  print("Los puntos finales son: x ",ini[0],"y ",ini[1],"z ",ini[2]," ")
  print("Fin de la rutina")


"Rutinas a dibujar"

# Cargar Marcador.
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


# Iniciales JA.

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

    

#Circunferencia
def circulo(a,b,r):
    #dibujar circulo
    curva(a,b,r,zoff,closed,200,1)#superior.
    curva(a,b,r,zoff,closed,200,0)#inferior
    #ir a punto de espera.
    Mover(puntoespera)
    
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


if __name__ == '__main__':
    try:
        interfaz()
    except rospy.ROSInterruptException:
        pass

