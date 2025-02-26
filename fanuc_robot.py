import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
# robotics toolbox"

import roboticstoolbox as rtb



cobot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.127,    a=0.0,    alpha=0,         qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0.33,  a=0.0,    alpha=-3.1416/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=-3.1416/2, d=0, a=0.28,   alpha=0,    qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0, a=0, alpha=-3.1415/2,     qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0.29, a=0.0,     alpha=3.1415/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=3.1416/2, d=0, a=0.07,    alpha=0,  qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=3.1416/2, d=0, a=0.0,    alpha=3.1415/2,         qlim=[-3.14, 3.14])
], name='Cobot UR10', base=SE3(0, 0, 0))

print(cobot)

#para modificar los angulos comodamente

joint1 = np.deg2rad(30)
joint2 = np.deg2rad(-30)
joint3 = np.deg2rad(-60)
joint4 = np.deg2rad(40)
joint5 = np.deg2rad(90)
joint6 = np.deg2rad(58)
joint7 = np.deg2rad(110)

T04DH = cobot.fkine([joint1, joint2, joint3, joint4, joint5, joint6, joint7])
print(T04DH)

T04DH_all = cobot.fkine_all([joint1, joint2, joint3, joint4, joint5, joint6, joint7])
print(T04DH_all[1]) #T01
print(T04DH_all[2]) #T02
print(T04DH_all[3]) #T03
print(T04DH_all[4]) #T04
print(T04DH_all[5]) #T05
print(T04DH_all[6]) #T06
print(T04DH_all[7]) #T07

q = np.array([
    [0, 0, 0, 0, 0, 0, 0],                           # 1. Todos en cero
    [joint1, 0, 0, 0, 0, 0, 0],                      # 2. Primer ángulo
    [joint1, joint2, 0, 0, 0, 0, 0],                 # 3. Segundo ángulo
    [joint1, joint2, joint3, 0, 0, 0, 0],            # 4. Tercer ángulo
    [joint1, joint2, joint3, joint4, 0, 0, 0],       # 5. Cuarto ángulo
    [joint1, joint2, joint3, joint4, joint5, 0, 0],  # 6. Quinto ángulo
    [joint1, joint2, joint3, joint4, joint5, joint6, 0],  # 7. Sexto ángulo
    [joint1, joint2, joint3, joint4, joint5, joint6, joint7],  # 8. Séptimo ángulo
    [joint1, joint2, joint3, joint4, joint5, joint6, 0],  # 9. Regresando
    [joint1, joint2, joint3, joint4, joint5, 0, 0],
    [joint1, joint2, joint3, joint4, 0, 0, 0],
    [joint1, joint2, joint3, 0, 0, 0, 0],
    [joint1, joint2, 0, 0, 0, 0, 0],
    [joint1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0]                            # 15. Todos en cero de nuevo
])

cobot.plot(q=q, backend = 'pyplot',dt = 1 , limits=[-1, 1, -1, 1, 0, 2], shadow =True,  jointaxes = True, )

q1 =np.array([0, 0, 0, 0, 0, 0, 0])
cobot.teach(q1)