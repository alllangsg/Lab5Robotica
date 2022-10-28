'Cinematica Inversa PhantomX'

import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

pi=np.pi

# pasar a radianes.
def rad(degrees):
    angle=degrees*np.pi/180
    return angle

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

# función para generar una trayectoria (loop de Pos())
def linea(ini,fin): #en cm y grados.
    #ini=[x,y,z,phid,q5d]
    #fin=[x,y,z,phid,q5d]
    # Se asume que ini[3]=fin[3]

    dt=0.1 #1mm paso entre puntos.
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

    #Publicar trayectoria al robot: 
    for x in range(0,len(Result)):
        joint_publisher(Result[x])

    return Result #cada fila de Result son los qs para un punto de la trayectoria.

"Rutinas a dibujar"
# Cargar Marcador.
def marcador():
    de=[20,20,20,0,0]
    a=[20,20,10,0,45]
    linea(de,a)
    
# Arco Inferior
# Arco Superior.
# Iniciales JA.
# Triangulo Equilatero.
# Circunferencia.
# 3 Lineas Paralelas.
# Descargar marcador.


#array de rutinas.
#postura=[home,pos1,pos2,pos3,pos4,rest]

#posturas de prueba:

# Función de publicación:
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
        if key == 'a':
            #marcador
            marcador()
            key = ' '
        elif key == 'w':
            #arco inf
            key = ' '
        elif key == 's':
            #arco sup.
            key = ' '
        elif key == 'd':
            #inicial J
            #inicial A
            key = ' '
        elif key == 'q':
            #Trianglulo equilatero.
            key = ' '
        elif key == 'e':
            #Circunferencia.
            key = ' '
        elif key =='r':
            #3 lineas paralelas
            key== ' '
        elif key =='f':
            #descargar marcador
            key== ' '


if __name__ == '__main__':
    try:
        interfaz()
    except rospy.ROSInterruptException:
        pass

