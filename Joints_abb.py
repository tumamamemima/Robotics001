import sympy as sp
from sympy.matrices import rot_axis3
# Para el ejemplo donde generamos la matriz DH
from spatialmath import *
from spatialmath.base import *
# Para poder Graficar
import matplotlib.pyplot as plt
import numpy as np
# robotics toolbox"
import numpy as np
import roboticstoolbox as rtb



cobot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.615,  a=0.1,    alpha=3.1416/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=3.1416/2, d=0, a=0.705,   alpha=0,    qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0, a=0.135, alpha=3.1415/2,     qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0.755, a=0.0,     alpha=-3.1415/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0, a=0,    alpha=3.1415/2,  qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0.085, a=0.0,    alpha=0,         qlim=[-3.14, 3.14])
], name='Abb_IRB_2400', base=SE3(0, 0, 0))

cobot.teach([0, 0, 0, 0, 0, 0])

T = np.array([
    [ 0.00,  0.00,  0.00,  0.00,  0.00,  0.00],
    [ 0.50, -0.50,  0.30, -0.30,  0.50, -0.50],
    [-0.40,  0.70, -0.80,  0.80, -0.90,  1.00],
    [ 1.00, -1.00,  1.00, -1.00,  0.50, -0.50],
    [-0.80,  0.80, -0.80,  0.80, -0.80,  0.80],
    [ 0.90, -0.90,  0.90, -0.90,  0.60, -0.60],
    [-1.00,  1.00, -1.00,  1.00, -0.70,  0.70],
    [ 1.00, -1.00,  1.00, -1.00,  1.00, -1.00],
    [-1.00,  1.00, -1.00,  1.00, -1.00,  1.00],
    [ 0.60, -0.60,  0.60, -0.60,  0.60, -0.60],
    [-0.60,  0.60, -0.60,  0.60, -0.60,  0.60]
])


# Define the number of points per segment and compute the dt per segment (5 sec per segment)
n_points = 20
dt_segment = 5 / (n_points - 1)

# Generate a continuous joint-space trajectory with jtraj
# Here we assume T is an array of joint configurations with shape (N, 6)
traj_list = []
for i in range(len(T) - 1):
    segment = rtb.jtraj(T[i], T[i+1], n_points).q
    # Remove the duplicate endpoint of each segment except the last one:
    if i < len(T) - 2:
        traj_list.append(segment[:-1])
    else:
        traj_list.append(segment)
        
traj = np.vstack(traj_list)

cobot.plot(q=traj, backend='pyplot', block=True)
