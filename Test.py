#%%
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
zoff=10 #altura marcador en cm.
ng=7 #numero puntos por trayectoria.
ncurv=10 #numero de puntos para curva 180°
radcir= 3 #radio circulo en cm.
lonlin= 5 #longitud lineas en cm.
alturaF= 4 #Altura de la F en cm. q
anchoF = 4 #ancho F en cm
ladotrian= 5 #lados del triangulo.
short=6 #distancia para bajar/ #puntos tray corta.
shorter=3

#posiciones cartesianas de interes:
limiteinf=[9,-9,zoff+3,closed]
limitesup=[21,-20,zoff+5,closed]
homex=[25,0,20,open]
puntoespera=[20,5,zoff+short,closed]

posmarcador=[14.4,-23.5,zoff+6,open]
tomarmarcador=[14.5,-23.5,zoff-3,closed]

triangorg=[20,0,zoff,closed]
centroCir=[20,5]
paralelasorg=[15,-5,zoff,closed]
posiniciales=[15,-15,zoff,closed]
test=[25,5,zoff,open]
libreorg=[10,10,zoff,closed]
puntosorg=[10,-10,zoff+short,closed]


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

#unir dos arrays
def unir(target,arr2):
    aux=asignar(target)
    for i in range(len(arr2)):
        aux.append(arr2[i])
    return aux

#imprimir un array
def imprimir(arr):
    for x in range(len(arr)):
        print(arr[x])
    return arr

#graficar.
def graficar(puntos):
    x=[]
    y=[]
    for i in range(len(puntos)):
        x.append(puntos[i][0])
        y.append(puntos[i][1])
        
    plt.figure(1)
    plt.plot(x,y)
    plt.show()
    
#nueva función linea recta.
def linea(ini,fin,n):
  pasox = (fin[0]-ini[0])/n
  #print("El paso en X es", pasox)
  pasoy = (fin[1]-ini[1])/n
  #print("El paso en Y es", pasoy)
  pasoz = (fin[2]-ini[2])/n
  #print("El paso en Z es", pasoz)
  #print("Ahora vamos a enviar cada punto de la línea")
  print("Puntos originales:",ini[0]," ",ini[1]," ",ini[2]," ")
  print("Puntos objetivo:",fin[0]," ",fin[1]," ",fin[2]," ")
  Result=[ini]
  for i in range(n):
    #print("Punto: ", i)
    ini[0] = ini[0]+pasox
    ini[1] = ini[1]+pasoy
    ini[2] = ini[2]+pasoz
    aux=asignar(ini)
    Result.append(aux)
    #Result=Pos(ini)
    #joint_publisher(Result)
    print("Nuevos puntos en trayectoria",ini[0]," ",ini[1]," ",ini[2]," ")
    #print("Repite ciclo")
  print("Los puntos finales son: x ",ini[0],"y ",ini[1],"z ",ini[2]," ")
  print("Fin de la rutina")
  return Result
#%%
#Triangulo Equilatero.   
def Newtriangulo():
    punto1 = [20,-1-1,zoff-0.5,closed]
    punto2 = [20,3,zoff-0.5,closed]
    punto3 = [23.46,1,zoff,closed]
    punto4 = [20,-1,zoff,closed]
    #puntos=[punto1,punto2,punto3,punto1]
    lin1=linea(punto1,punto2,5)
    lin2=linea(punto2,punto3,5)
    lin3=linea(punto3,punto4,5)
    aux=unir(lin1,lin2)
    graficar(aux)
    aux=unir(aux,lin3)
    graficar(aux)
    print(aux[0])

Newtriangulo()
#%%
#punto equidistantes.
def puntos():
    #moverse al origen
    graf=[puntosorg]
    aux=asignar(puntosorg)
    #hacer puntos
    for i in range(5):
        #bajar(aux,1)
        #bajar(aux,0)
        aux[0]+=2
        graf.append(aux)

    graficar(graf)
    #a punto de espera.
    #Mover(puntoespera)
puntos()
# %%
#trayectoria curva
def curva(a,b,r,z,q5d,thetamaxd,graf):
    #centro (a,b) en #cm
    # r en cm.
    centro=[a,b,z,q5d]
    #ir al centro
    cir=[centro]
    
    q5=rad(q5d) #apertura del gripper
    thetamax=rad(thetamaxd)
    
    n=int(15*thetamaxd/180) #No. de puntos.
    print(n)
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

    #graficar circulo.
    plt.figure(1)
    plt.plot(x,y)
    plt.show()
        
    #ir a pto inicio circulo (mitad izq)
    inicio=asignar(centro)
    inicio[0]+=r
    #bajar(centro,0)
    cir.append(inicio)

    #Dibujar circulo
    Result=[x[0],y[0],z,q5]
    cir.append(Result)
    for i in range(1,n):
        result2=[x[i], y[i], z, q5]
        cir.append(result2)

    #graficar puntos guardados en cir.
    graficar(cir)
    grf=[]
    for i in range(len(cir)-1):
        grf=unir(grf,linea(cir[i],cir[i+1],1))
    
    #print(grf[0])
    
    if graf==1:
        return(grf)
    else:
        graficar(grf)

def circulo(a,b,r):
    #dibujar circulo
    curva(a,b,r,zoff,closed,380,0)
    #ir a punto de espera.
    #Mover(puntoespera)
    return

#Circunferencia.
circulo(centroCir[0],centroCir[1],radcir) 
# %%
    
def lineas(n):
    #Mover(paralelasorg) #15,-5
    punto1=[15,0,zoff,closed]
    gf=[paralelasorg]
    lin1=linea(paralelasorg,punto1,n)
    gf=unir(gf,lin1)
    #bajar(punto1,0)
    #linea 2
    punto2=[17,-5,zoff,closed]
    punto3=[17,0,zoff,closed]
    #acercarse a punto 2 y bajar
    aux=asignar(punto2)
    aux[2]+=2
    #Mover(punto2)
    #bajar(punto2,1)
    #hacer linea
    lin2=linea(punto2,punto3,n)
    gf=unir(gf,lin2)
    #bajar(punto3,0)
    #linea 3
    punto4=[19,-5,zoff,closed]
    punto5=[19,0,zoff,closed]
    #acercarse y bajar marcador
    aux=asignar(punto4)
    aux[2]+=2
    #Mover(aux)
    #bajar(punto4,1)
    #dibujar linea
    lin3=linea(punto4,punto5,n)
    gf=unir(gf,lin3)
    #ir a punto de espera.
    #Mover(puntoespera)
    graficar(gf)

lineas(3)

# %%
#iniciales
def iniciales():
    posiniciales=[20,20,zoff,closed]
    gf=[posiniciales]
    #la A:
    #aproximacion
    #Mover(posiniciales)
    #auxi=asignar(posiniciales)
    #auxi[2]+=zoff
    #Mover(auxi)
    #bajar(posiniciales,1,0)
    #linea 1
    punto1=[16,13,zoff,closed]
    #Mover(punto1)
    aux=linea(posiniciales,punto1,4)
    gf=unir(gf,aux)
    #linea 2
    punto2=[10,11,zoff,closed]
    aus2=linea(punto1,punto2,4)
    gf=unir(gf,aus2)
    #Mover(punto2)
    #linea 3

    #la F:
    #ir a espera
    #bajar(punto2,0,0)
    #Mover(puntoespera)
    graficar(gf)

iniciales()

# %%
#figura libre
def libre():
    #xl=22
    #yl=-2
    gf=[]
    posLibre=[22-0.6,-2,zoff-0.5,closed]
    #Mover(posLibre)
    #cruz
    #linea1:
    pos1=[22+4,-2,zoff-0.5,closed]
    aux=linea(posLibre,pos1,6)
    gf=unir(gf,aux)
    #bajar(pos1,0,0) #subir
    #linea2:
    pos2=[22+2,-2-2,zoff,closed]
    pos3=[22+2,-2+2,zoff,closed]
    #Mover(pos2)
    gf.append(pos2)
    aux1=linea(pos2,pos3,6)
    gf=unir(gf,aux1)
    #bajar(pos3,0,0)#subir

    #curva superior
    #poscurva=[22+2,-2-2,zoff,closed]
    #Mover(poscurva)
    #dibujar semicirculo
    aux2=curva(24,-2,2,zoff,closed,190,1)
    gf=unir(gf,aux2)
    #posicion de espera:
    #Mover(puntoespera)
    graficar(gf)

libre()
# %%
