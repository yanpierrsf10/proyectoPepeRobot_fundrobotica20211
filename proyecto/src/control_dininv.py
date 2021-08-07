#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages

import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
fqact = open("/tmp/qactual.txt", "w")
fqdes = open("/tmp/qdeseado.txt", "w")
fxact = open("/tmp/xactual.txt", "w")
fxdes = open("/tmp/xdeseado.txt", "w")

# Nombres de las articulaciones
jnames = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, 0.05, 0, 0, -1, 1, 0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0., 0.])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
qdes = np.array([0, 0.05, 0, 0, -pi/2, pi/2, 0])
# Velocidad articular deseada
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0])
# Aceleracion articular deseada
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes)[0:3,3]
# Velocidad resultante de la configuraciion articular deseada
dxdes = np.zeros(3)
# Aceleracion resultante de la configuracion articular deseada
ddxdes = np.zeros(3)
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/robot.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Bucle de ejecucion continua
t = 0.0

# Se definen las ganancias del controlador
valores = 0.5*np.array([1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Arrays a usar 
M = np.zeros([ndof, ndof]) 
b = np.zeros(ndof)
Ja_pass = np.zeros([3, ndof])

while not rospy.is_shutdown():    
    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Restricciones articulares
    
    if (q[0] >= 3.1):
        q[0] = 3.1
        print("q1 maximo alcanzado")
    if (q[0] <= -3.1):
        q[0] = -3.1
        print("q1 minimo alcanzado")
    if (q[1] >= 0.5):
        q[1] = 0.5
        print("q2 maximo alcanzado")
    if (q[1] <= 0):
        q[1] = 0
        print("q2 minimo alcanzado")
    if (q[2] >= 3.1):
        q[2] = 3.1
        print("q3 maximo alcanzado")
    if (q[2] <= -1.35):
        q[2] = -1.35
        print("q3 minimo alcanzado")
    if (q[3] >= 3.1):
        q[3] = 3.1
        print("q4 maximo alcanzado")
    if (q[3] <= -3.1):
        q[3] = -3.1
        print("q4 minimo alcanzado")
    if (q[4] >= 0):
        q[4] = 0
        print("q5 maximo alcanzado")
    if (q[4] <= -3.1):
        q[4] = -3.1
        print("q5 minimo alcanzado")
    if (q[5] >= 3.1):
        q[5] = 3.1
        print("q6 maximo alcanzado")
    if (q[5] <= 0):
        q[5] = 0
        print("q6 minimo alcanzado")
    if (q[6] >= 3.1):
        q[6] = 3.1
        print("q7 maximo alcanzado")
    if (q[6] <= -3.1):
        q[6] = -3.1
        print("q7 minimo alcanzado")
    # Posicion actual del efector final
    x = fkine(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------

    # Matriz de Inercia 
    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
    # Vector de efectos no lineales
    rbdl.NonlinearEffects(modelo, q, dq, b)
    # Jacobiano de poscicion
    Ja = jacobian(q)
    # Derivada del Jacobiano de posicion
    dJa = (Ja - Ja_pass)/dt
    Ja_pass = Ja
    # Error
    e = xdes - x
    dx = Ja.dot(dq)
    de = dxdes - dx
    # Ley de control
    u = M.dot(np.linalg.pinv(Ja)).dot(ddxdes - dJa.dot(dq) + Kd.dot(de) + Kp.dot(e)) + b

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t + dt

    print('X des:'); print(np.round(xdes,2))
    print('X act:'); print(np.round(x,2))

    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
