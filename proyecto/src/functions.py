import numpy as np
from copy import copy

#import rbdl
# Files for the logs
ecurrent = open("/tmp/ecurrent.txt", "w") 

cos=np.cos; sin=np.sin; pi=np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/robot.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq



def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    cth = np.cos(theta);    sth = np.sin(theta)
    ca = np.cos(alpha);  sa = np.sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    return T
    
    

def fkine(q):
    """
    Calcular la cinematica directa del robot Pepe dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)
    l1=0.5
    l2=1.2
    l3=0.21
    l4=0.385371
    l5=1.204
    l6=0.4225
    l7=0.3145
    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T01 = dh(       l1,           q[0],     0,           0)
    T12 = dh(    l2+q[1],            0,     0,     np.pi/2)
    T23 = dh(       l3,   np.pi/2+q[2],    l4,     np.pi/2)
    T34 = dh(       l5,           q[3],     0,     np.pi/2)
    T45 = dh(        0,           q[4],   -l6,           0)
    T56 = dh(        0,     np.pi+q[5],     0,     np.pi/2)
    T67 = dh(       l7,           q[6],     0,           0)
    
    # Efector final con respecto a la base
    T = T01.dot(T12).dot(T23).dot(T34).dot(T45).dot(T56).dot(T67)
    return T

def jacobian(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma
    como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x7
    J = np.zeros((3,7))
    # Transformacion homogenea inicial (usando q)
    tf = fkine(q)
    TF = tf[0:3,3]
    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        deltav = np.zeros(7)
        deltav[i] = delta
        xq = tf[0:3,3]
        # Incrementar la articulacion i-esima usando un delta
        qd = dq + deltav
        # Transformacion homogenea luego del incremento (q+delta)
        xqd = fkine(qd)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[:,i] = (xqd[0:3,3]-xq)/delta  
    return J

def ikine(xdes, q0):
    """
    Calcular la cinematica inversa numericamente a partir de la
    configuracion articular inicial de q0.
    Emplear el metodo de newton
    """
    epsilon = 0.001
    max_iter = 100
    delta = 0.00001
    q = copy(q0)
    ee = []
    iteraciones=0
    for i in range(max_iter):
        J = jacobian(q, delta)
        f = fkine(q)[0:3,3]
        e = xdes-f
        #Log error Values
        ecurrent.write(str(e[0])+' '+str(e[1]) +' '+str(e[2])+'\n')
        #New joints values 
        q = q + np.dot(np.linalg.pinv(J), e)
        # Norma del error
        enorm = np.linalg.norm(e)
        ee.append(enorm) # Almacena los errores
        iteraciones+=1
        if (np.linalg.norm(e) < epsilon):
            print('Iteraciones totales:')
            print(iteraciones)
            print('Error en la iteracion {}: {}'.format(iteraciones, np.round(np.linalg.norm(e),4)))
            print('Valores articulares')
            print(str(np.round(q,3)) + '[rads]')
            break
    return q

def ik_gradient(xdes, q0):
    """
    Calcular la cinematica inversa de Pepe numericamente a partir de la
    configuracion articular inicial de q0.
    Emplear el metodo gradiente
    """
    epsilon = 0.001
    max_iter = 10000
    delta = 0.00001
    alpha =0.05
    iteraciones=0
    q = copy(q0)
    for i in range(max_iter):
        J = jacobian(q, delta)
        f = fkine(q)[0:3,3]
        e = xdes-f
        #Log error Values
        ecurrent.write(str(e[0])+' '+str(e[1]) +' '+str(e[2])+'\n')
        q = q + alpha*np.dot(J.T, e)
        iteraciones+=1
        if (np.linalg.norm(e) < epsilon):
            print('Iteraciones totales:')
            print(iteraciones)
            print('Error en la iteracion {}: {}'.format(iteraciones, np.round(np.linalg.norm(e),4)))
            break
    print('Valores articulares')
    print(str(np.round(q,3)) + '[rads]')
    return q

def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,7))
    # Implementar este Jacobiano aqui
    T1 = fkine(q)
    Pos_Ori = TF2xyzquat(T1)
    for i in xrange(7):
        dq = copy(q)
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+dq)
        Td = fkine(dq)
        Pos_Ori_d = TF2xyzquat(Td)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[:, i] = (Pos_Ori_d.T - Pos_Ori.T)/ delta
    return J

def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R