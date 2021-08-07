import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/robot.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6, 0.5])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0, 0.8])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5, 0.2])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(ndof)            # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

print('INCISO 3.3')

# Vector de gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print('Vector de gravedad [g]:'); print(np.round(g, 2))
# Vector de Coriolis/Centrifuga
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c - g
print('Vector de Coriolis/Centrifuga [c]:'); print(np.round(c, 2))
# Matriz de inercia
mi = np.zeros(ndof)
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i, :], mi)
    M[:, i] = mi - g
print('Matriz de Inercia [M]:'); print(np.round(M, 2))

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

print('INCISO 3.4')

# Matriz de inversa obtenida directamente
rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
print('Matriz de Inercia obtenida directamente [M2]:'); print(np.round(M2, 2))
# Vector de efectos no lineales
rbdl.NonlinearEffects(modelo, q, dq, b2)
print('Vector de efectos no lineales [b2]:'); print(np.round(b2, 2))

# Parte 2: Verificacion de valores
print('INCISO 3.5')
# Para la matriz de inercia
M_comp = M - M2
print('Para la Matriz de Inercia: M(q) - M2(q)'); print(np.round(M_comp,2))
# Para los efectos no lineales
b_comp = (c + g) - b2
print('Para el Vector de efectos no lineales: c(q, dq) + g(q) - b2(q, dq)'); print(np.round(b_comp,2))

# Parte 3: Verificacion de la expresion de la dinamica
print('INCISO 3.6')
# Vector de torques obtenido directamente
print('Vector de torques obtenido directamente'); print(np.round(tau,2))
# Vector de torques obtenido: M(q)ddq + c(q,dq) + g(q)
tau1 = M.dot(ddq) + c + g
print('Vector de torques obtenido: M(q)ddq + c(q,dq) + g(q)'); print(np.round(tau1,2))
# Vector de torques obtenido: M2(q)ddq + b2(q,dq)
tau2 = M2.dot(ddq) + b2
print('Vector de torques obtenido: M(q)ddq + c(q,dq) + g(q)'); print(np.round(tau2,2))