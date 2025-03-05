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
    rtb.RevoluteDH(d=0.0,    a=0.0,    alpha=0,         qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0.128,  a=0.0,    alpha=3.1416/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=3.1416/2, d=0.176, a=0.612,   alpha=-3.1415,    qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0.1639, a=0.5716, alpha=3.1415,     qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=3.1416/2, d=0.1639, a=0.0,     alpha=3.1415/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0.1157, a=0.0,    alpha=-3.1415/2,  qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0.0922, a=0.0,    alpha=0,         qlim=[-3.14, 3.14])
], name='Cobot UR10', base=SE3(0, 0, 0))

print(cobot)

#cobot.teach([0, 0, 0, 0, 0, 0,0])
import numpy as np


T = np.array([
    [-0.075,  0.075,  0.075 + 0.2],  # A  -> [-0.075, 0.075, 0.275]
    [ 0.075,  0.075,  0.075 + 0.2],  # B  -> [0.075, 0.075, 0.275]
    [ 0.075, -0.075,  0.075 + 0.2],  # C  -> [0.075, -0.075, 0.275]
    [-0.075, -0.075,  0.075 + 0.2],  # D  -> [-0.075, -0.075, 0.275]
    [-0.075,  0.075,  0.075 + 0.2],  # A  -> [-0.075, 0.075, 0.275]
    [-0.075,  0.075, -0.075 + 0.2],  # E  -> [-0.075, 0.075, 0.125]
    [ 0.075,  0.075, -0.075 + 0.2],  # F  -> [0.075, 0.075, 0.125]
    [ 0.075, -0.075, -0.075 + 0.2],  # G  -> [0.075, -0.075, 0.125]
    [-0.075, -0.075, -0.075 + 0.2],  # H  -> [-0.075, -0.075, 0.125]
    [-0.075, -0.075,  0.075 + 0.2],  # I  -> [-0.075, -0.075, 0.275]
    [-0.075,  0.075,  0.075 + 0.2]   # J  -> [-0.075, 0.075, 0.275]
])
print(T)


via  = np.empty((0,3))

for punto in T:
    xyz = np.array(punto)
    #print(xyz)
    via = np.vstack((via, xyz)) #append filas a puntos en via 
    
xyz_traj = rtb.mstraj(via, qdmax =[0.5,0.5,0.5], dt=0.02 , tacc=0.2).q
#hace cubo
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.plot(xyz_traj[:,0], xyz_traj[:,1], xyz_traj[:,2])
ax.scatter(xyz_traj[0,0], xyz_traj[0,1], xyz_traj[0,2], c='r' , marker='*')
ax.scatter(xyz_traj[-1,0], xyz_traj[-1,1], xyz_traj[-1,2], c='b' , marker='o')
plt.show()
#hace trajectoria
T_tool = SE3.Trans(0.4,0,0.0)*SE3.Trans(xyz_traj)*SE3.OA([0,-1,0],[1,0,0])
we = [1,1,1,0,0,0]
sol = cobot.ikine_LM(T_tool, q0=[0,0,0,0,0,0,0], ilimit=100, slimit=100, tol=1e-6 , mask = we)
print(sol)
cobot.plot(q=sol.q, limits=[-0.3,0.6,-0.6,0.6,-0.1,1],
backend='pyplot', shadow = True, jointaxes = True, eeframe = True, block = True)


# Compute and plot individual inverse kinematics solutions for each via point
ik_solutions = []  # list to hold the IK solutions
we = [1, 1, 1, 0, 0, 0]  # mask: only position control

for punto in via:
    # Create the tool transformation for the current via point.
    # Here we add a fixed translation (0.4 m along x) plus the via point translation,
    # and a fixed orientation defined by SE3.OA.
    T_tool_pt = SE3.Trans(0.4, 0, 0.0) * SE3.Trans(punto) * SE3.OA([0, -1, 0], [1, 0, 0])
    
    # Compute the inverse kinematics solution for the current point.
    sol = cobot.ikine_LM(
        T_tool_pt,
        q0=[0, 0, 0, 0, 0, 0, 0],  # initial guess for the 7 joints
        ilimit=100, slimit=100, tol=1e-6,
        mask=we
    )
    
    # Save the solution if needed
    ik_solutions.append(sol.q)
    
    # Plot the current configuration (each plot is independent, without a trajectory)
    cobot.plot(
        q=sol.q,
        limits=[-0.3, 0.6, -0.6, 0.6, -0.1, 1],
        backend='pyplot',
        shadow=True,
        jointaxes=True,
        eeframe=True,
        block=False
    )
    plt.pause(0.5)  # Pause briefly to view each configuration

# Keep the final plot window open
plt.show()
