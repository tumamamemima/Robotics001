import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
#definir simbolos
theta, d , a , alpha = sp.symbols('theta ,d ,a, alpha',)

# Creamos nuestra matriz T
T = sp.Matrix([
    [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
    [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
    [           0,             sp.sin(alpha),              sp.cos(alpha),               d],
    [           0,                       0,                         0,                  1]
])

# sp.pprint(T)

# Definimos los ángulos y distancias para que cada uno sea único
theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7 = sp.symbols('theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7 ')
#de la tabla
print("T01")
T01 = T.subs({d: 0, a: 0, alpha: 0})
T01 = T01.subs({theta: theta_1})
sp.pprint(T01)

print("T12")
T12 = T.subs({d: 0.128, a: 0, alpha: sp.pi/2})
T12 = T12.subs({theta: theta_2})
sp.pprint(T12)

print("T23")
T23 = T.subs({ theta: sp.pi/2, d: 0.176, a: 0.612, alpha: -sp.pi })
T23 = T23.subs({theta: theta_3})
sp.pprint(T23)

print("T34")
T34 = T.subs({ d: 0.1639, a: 0.5716, alpha: sp.pi})
T34 = T34.subs({theta: theta_4})
sp.pprint(T34)

print("T45")
T45 = T.subs({ theta: sp.pi/2, d: 0.1639, a: 0, alpha: sp.pi/2 })
T45 = T45.subs({theta: theta_5})
sp.pprint(T45)

print("T56")
T56 = T.subs({d: 0.1157, a: 0, alpha: -sp.pi/2}) 
T56 = T56.subs({theta: theta_6})
sp.pprint(T56)

print("T67")
T67 = T.subs({d: 0.0922, a: 0, alpha: 0 })
T67 = T67.subs({theta: theta_7})
sp.pprint(T67)

print("T07 simplified")
T07 = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67
T07_s = T07.applyfunc(sp.simplify)
sp.pprint(T07_s)

# insertar valores de joints

joint1 = np.deg2rad(0)
joint2 = np.deg2rad(0)
joint3 = np.deg2rad(0)
joint4 = np.deg2rad(0)
joint5 = np.deg2rad(0)
joint6 = np.deg2rad(0)
joint7 = np.deg2rad(0)

#solve home matrix
print("home matrix ==")
T07_solved = T07_s.subs({theta_1: joint1, theta_2: joint2, theta_3: joint3, theta_4: joint4, theta_5: joint5, theta_6: joint6, theta_7: joint7})
sp.pprint(T07_solved)

#graficar frames
T0 = rotz(0, unit ='deg')
trplot( T0 ,lenght = 0.1, color='k',frame = '0')

T01_n = T01.subs({theta_1: joint1})
T01_n = np.array(T01_n).astype(np.float64)
trplot( T01_n ,lenght = 0.1, color='b',frame = '1')

T12_n = T12.subs({theta_2: joint2})
T02_n=T01_n @ T12_n
T02_n = np.array(T02_n).astype(np.float64)
trplot( T02_n ,lenght = 0.1, color='r',frame = '2')

T23_n = T23.subs({theta_3: joint3})
T03_n=T02_n @ T23_n
T03_n = np.array(T03_n).astype(np.float64)
trplot( T03_n ,lenght = 0.1, color='g',frame = '3')

T34_n = T34.subs({theta_4: joint4})
T04_n=T03_n @ T34_n
T04_n = np.array(T04_n).astype(np.float64)
trplot( T04_n ,lenght = 0.1, color='y',frame = '4')
T45_n = T45.subs({theta_5: joint5})

T05_n=T04_n @ T45_n
T05_n = np.array(T05_n).astype(np.float64)
trplot( T05_n ,lenght = 0.1, color='c',frame = '5')

T56_n = T56.subs({theta_6: joint6})
T06_n=T05_n @ T56_n
T06_n = np.array(T06_n).astype(np.float64)
trplot( T06_n ,lenght = 0.1, color='m',frame = '6')

T67_n = T67.subs({theta_7: joint7})
T07_n=T06_n @ T67_n
T07_n = np.array(T07_n).astype(np.float64)
trplot( T07_n ,lenght = 0.1, color='r',frame = '7')

Tf_n = np.array(T07_solved).astype(np.float64)
trplot( Tf_n ,lenght = 0.1, color='b',frame = 'F')


plt.grid(True)
plt.title('Robot UR10')
plt.axis('equal')
plt.show(block = True)