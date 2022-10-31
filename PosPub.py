'Cinematica Inversa PhantomX'

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#constantes de interes.
pi=np.pi
open=0
closed=45 #angulo para gripper cerrado. °
phig=0 #orientación de la muñeca.
zoff=11 #altura marcador en cm.
dt=0.5 #5mm entre puntos.
radcir= 4 #radio circulo en cm.
lonlin= 3 #longitud lineas en cm.
alturaF= 4 #Altura de la F en cm. q
anchoF = 4 #ancho F en cm
ladotrian= 4 #lados del triangulo.

# pasar a radianes.
def rad(degrees):
    angle=degrees*np.pi/180
    return angle

#posiciones cartesianas de interes:
homex=[20,0,20,phig,open]
limiteinf=[0,-20,zoff,phig,closed]
limitesup=[0,-30,zoff,phig,closed]
posmarcador=[20,20,zoff+5,phig,open]
tomarmarcador=[20,20,zoff,phig,closed]
triangorg=[10,10,zoff,phig,closed]
centroCir=[-5,20]
paralelasorg=[-10,20,zoff,phig,closed]
puntoespera=[-10,10,zoff+10,phig,closed]
posiniciales=[10,15,zoff,phig,closed]

# posturas de interes:
home=[0,0,0,0,0]
homec=[0,0,0,0,rad(closed)]
espera=[10,-10,10,0,rad(closed)]



# multiplicar matrices:
def mul_matrix(A,B):
  global C
  if  A.shape[1] == B.shape[0]:
    C = np.zeros((A.shape[0],B.shape[1]),dtype = float)
    for row in range(A.shape[0]): 
        for col in range(B.shape[1]):
            for elt in range(len(B)):
              C[row, col] += A[row, elt] * B[elt, col]
    return C
  else:
    return "Sorry, cannot multiply A and B."

#delay entre trayectorias.
def delayros():
    rospy.sleep(10)

# función de cinemática inversa.
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
    Solgof= [q1,q2+alpha2,q3-alpha2,q4,q5] #rad
    return Solgof

#MTH para DH desde el marco i-1 al marco i.#rad
def MTHDH(ai,alphai,di,thetai):
    H = [[np.cos(thetai),-np.sin(thetai)*np.cos(alphai), np.sin(thetai)*np.sin(alphai), ai*np.cos(thetai)],[np.sin(thetai),np.cos(thetai)*np.cos(alphai),-np.cos(thetai)*np.sin(alphai),ai*np.sin(thetai)],[0, np.sin(alphai), np.cos(alphai), di],[0, 0, 0, 1]]
    return H

#función cinemática directa: 
def Direct(input):
  q1=input[0]
  q2=input[1]
  q3=input[2]
  q4=input[3]
  q5=input[4]
  
  phid=q2+q3+q4
  alpha2=np.arccos(3.39/10.63) #radians
  MTH01 = np.array(MTHDH(0,np.pi/2,4.44,q1)) #rad 
  #MTH de O2 respecto a O1
  MTH12 = np.array(MTHDH(10.63,0,0,q2+alpha2))  #rad 
  #MTH de O3 respecto a O2
  MTH23 = np.array(MTHDH(10.5,0,0,q3-alpha2))  #rad
  #MTH de O4 respecto a O3
  MTH34 = np.array(MTHDH(12.8,0,0,q4))#rad
  #MTH de O5 respecto a O0.
  #MTH04=MTH01*MTH12*MTH23*MTH34
  aux=mul_matrix(MTH01,MTH12)
  aux=mul_matrix(aux,MTH23)
  aux=mul_matrix(aux,MTH34)
  result=[]
  result.append(aux[0][3])#x
  result.append(aux[1][3])#y
  result.append(aux[2][3])#z
  result.append(phid)#phid
  q5d=q5*180/np.pi #degrees
  result.append(q5d) #q5d
  return result #x,y,z,phid,q5


# función para generar una linea recta (loop de Pos())
def linea(ini,fin): #en cm y grados.
    #ini=[x,y,z,phid,q5d]
    #fin=[x,y,z,phid,q5d]
    # Se asume que ini[3]=fin[3], pero no es necesario.
    #número de puntos.
    long=np.sqrt((fin[0]-ini[0])**2 + (fin[1]-ini[1])**2 + (fin[2]-ini[2])**2)/dt
    long=int(long)
    #paso para cada dirección.
    pasox=(fin[0]-ini[0])/long
    pasoy=(fin[1]-ini[1])/long
    pasoz=(fin[2]-ini[2])/long

    #Matriz de puntos
    Result=[]
    for x in range(0,long):
        Result.append(Pos(ini[0]+pasox,ini[1]+pasoy,ini[2]+pasoz,ini[3],ini[4]))
        if x==long-1 and ini[4]!=fin[4]: #abrir o cerrar gripper al final
            Result[x,4]=fin[4]
        joint_publisher(Result[x]) #cada fila de Result son los qs para un punto de la trayectoria.

    last=Result[long-1]
    last=Direct(last)
    delayros()
    return last # ultimo punto.

#trayectoria curva
def curva(a,b,r,z,phid,q5d,thetamaxd):
    #centro (a,b) en #cm
    # Phi no cambia.
    # r en cm.
    phi=rad(phid) #orientacion de la muñeca.
    q5=rad(q5d) #apertura del gripper
    thetamax=rad(thetamaxd)
    
    n=int((thetamax*r)/dt) #No. de puntos.
    dth=(thetamax)/n #paso de angulo.
    #lista de angulos.
    theta=[0]
    for i in range(1,n):
        theta.append(theta[i-1]+(dth))
    
    #calculo de x y y.
    x=[]
    y=[]
    for i in range(0,n):
        x.append(r*(np.cos(theta[i]))+a)
        y.append(r*(np.sin(theta[i]))+b)

    #Matriz de puntos
    Result=[]
    for i in range(0,n):
        Result.append(Pos(x[i],y[i],z,phi,q5))
        joint_publisher(Result[x])
    
    delayros()
    last=Result[n-1]
    last=Direct(last)
    return last #ultimo punto.

"Rutinas a dibujar"

#ir a posicion de espera.
def espera_tray(last):
    Result=linea(last,puntoespera)
    return Result

# Cargar Marcador.
def marcador():
    #ir encima del marcador.
    last=linea(homex,posmarcador)

    #bajar hacia el marcador y cerrar griper.
    last=linea(last,tomarmarcador)

    #subir marcador
    last=linea(last,tomarmarcador)

    #ir a posicion de espera.
    last=espera_tray(last)
    return last
    
# Arco Inferior y Arco Superior.
def arcos(lim):
    #Acercarse a punta izq.
    de=puntoespera
    
    if lim==0: #arco inferior
        a=limiteinf
        r=4 #curvatura estimada en cm.
    elif lim==1: #arco superior.
        a=limitesup
        r=20 #curvatura estimada en cm.

    Approach=linea(de,a)
    Approach=Pos(Approach) #pose.

    #Dibujar Arco.
    n=int((pi*r)/dt) #No. de puntos.
    q10=rad(-90) #q1 inicial.
    dth=(pi)/n #paso de angulo.
    aux=Approach 
    Result=[Approach] #ultima pose dada.
    #cambiar q1 para cada punto.
    for i in range(1,n):
        q10=q10+dth
        aux[0]=q10
        Result.append(aux)
        joint_publisher(Result[i])

    last=Result[n-1]
    last=Direct(last)

    #ir a punto de espera:
    last=espera_tray(last)

    return last #ultimo punto.


# Iniciales JA.
def iniciales(centro):
    #centro es un punto
    #Acercarse a punto.
    last=linea(puntoespera,centro)

    #Dibujar F
    #Dibujar Vertical
    objetivo=last
    objetivo[1]=alturaF
    last=linea(last,objetivo)
    #Dibujar Horizontal
    objetivo=last
    objetivo[0]=anchoF
    last=linea(last,objetivo)
    #Dibujar Mitad
    #acercarse a mitad.
    objetivo=last
    objetivo[0]-=anchoF
    objetivo[1]-=alturaF/2
    objetivo[2]=zoff+3
    last=linea(last,objetivo)
    #bajar lapiz
    objetivo[2]=zoff
    last=linea(last,objetivo)
    #dibujar linea.
    objetivo=last
    objetivo[0]+=anchoF
    last=linea(last,objetivo)

    #Dibujar A
    #ir a punto A.
    objetivo=last
    objetivo[0]+=anchoF+1
    objetivo[1]-=alturaF/2
    objetivo[2]=zoff+3# subir marcador
    last=linea(last,objetivo) 
    #bajar marcador
    objetivo[2]=zoff
    last=linea(last,objetivo)
    #dibujar triangulo.
    objetivo=last
    objetivo[0]+=anchoF/2
    objetivo[1]+=alturaF
    last=linea(last,objetivo)
    objetivo=last
    objetivo[0]+=anchoF/2
    objetivo[1]-=alturaF
    last=linea(last,objetivo)
    #dibujar mitad.
    #ir a mitad
    objetivo=last
    objetivo[0]=anchoF/4
    objetivo[1]=alturaF/2
    objetivo[2]=zoff+3
    last=linea(last,objetivo)
    #bajar marcador
    objetivo[3]=zoff
    last=linea(last,objetivo)
    #dibujar
    objetivo=last
    objetivo[0]=3*anchoF/4
    last=linea(last,objetivo)

    #ir a punto de espera 
    last=espera_tray(last)
    return last  


# Triangulo Equilatero.
def triang(origin,a):
    #origin=[x,y,z,phid,q5d] punto inf izq.
    punto2=origin
    punto2[0]=origin[0]+a #inf. der.
    punto3=origin
    punto3[0]=origin[0]+a/2
    punto3[1]=origin[1]+(np.sqrt(3)*a/2) #punta sup.
    #Primer vector 0->2
    last=linea(origin,punto2)
    #Segundo vector 2->3
    last=linea(last,punto3)
    #Tercer Vector 3->0
    last=linea(last,origin)

    #ir a punto de espera
    last=espera_tray(last)
    return last

# Circunferencia.
def circulo(a,b,r):
    #dubujar circulo
    last=curva(a,b,r,zoff,phig,closed,360)
    #ir a punto de espera.
    last=espera_tray(last)
    return last

# 3 Lineas Paralelas.
def paralelas(n,sepy,m,origin):
    #origin=[x,y,z,phid,q5d] punto inf izq.
    #n= numero de lineas.
    #sep = separacion entre lineas.
    # m= slope
    l=lonlin #cm largo de las lineas.
    dx=np.sqrt(l**2/(m**2+1))
    dy=m*dx
    #ir al origen de las lineas.
    last=linea(puntoespera,origin)
    #dibujar n lineas:
    for i in range(n):
        #calcular puntos.
        origin[0]=origin[0]+1*i #desplaza 1cm x
        origin[1]=origin[1]+sepy*i #desplaza sepy cm y
        punto2=origin
        punto2[0]=origin[0]+dx #x2
        punto2[1]=origin[1]+dy #y2
        #Dibujar linea
        last=linea(next,origin) #bajar marcador
        last=linea(origin,punto2) #dibujar linea.
        #ir al siguiente origen
        current=punto2 #actualizar punto actual.
        next=origin #calcular siguien origen
        next[0]=origin[0]+1*(i+1) #desplaza 1cm x
        next[1]=origin[1]+sepy*(i+1) #desplaza sepy cm y
        next[2]=zoff+5 #alzar marcador.
        last=linea(current,next)
    #ir a punto de espera.
    last=espera_tray(last)
    return last

# Descargar marcador.
def descargue():
    #ir a pos marcador.
    objetivo=posmarcador
    objetivo[4]=closed
    last=linea(puntoespera,objetivo)
    #bajar marcador y soltar
    objetivo=tomarmarcador
    objetivo[4]=open
    last=linea(last,objetivo)
    #subir 3cm.
    objetivo=last
    objetivo[2]+=3
    last=linea(last,objetivo)

    #ir a home. 
    last=linea(last,homex)
    return last

#array de rutinas.
#postura=[home,pos1,pos2,pos3,pos4,rest]

#posturas de prueba:

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
    rospy.sleep(1)
    

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
            iniciales(posiniciales)
            key = ' '
        elif key == 'a':
            #Trianglulo equilatero.
            triang(triangorg,ladotrian)
            key = ' '
        elif key == 'd':
            #Circunferencia.
            circulo(8,8,radcir)
            key = ' '
        elif key =='x':
            #3 lineas paralelas
            #paralelas(n,sepy,m,origin):
            paralelas(3,0,2,paralelasorg)
            key== ' '
        elif key =='z':
            #descargar marcador
            descargue()
            key== ' '


if __name__ == '__main__':
    try:
        interfaz()
    except rospy.ROSInterruptException:
        pass

