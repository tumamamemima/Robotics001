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
    rtb.RevoluteDH(d=0.457,  a=0.0,    alpha=-3.1416/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=-3.1416/2, d=0, a=0.28,   alpha=0,    qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(d=0, a=0, alpha=-3.1415/2,     qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0.29, a=0.0,     alpha=3.1415/2,   qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0, a=0,    alpha=-3.1415/2,  qlim=[-3.14, 3.14]),
    rtb.RevoluteDH(offset=0, d=0.07, a=0.0,    alpha=0,         qlim=[-3.14, 3.14])
], name='FANUC', base=SE3(0, 0, 0))

print(cobot)

cobot.teach([0, 0, 0, 0, 0, 0])
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

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.plot(xyz_traj[:,0], xyz_traj[:,1], xyz_traj[:,2])
ax.scatter(xyz_traj[0,0], xyz_traj[0,1], xyz_traj[0,2], c='r' , marker='*')
ax.scatter(xyz_traj[-1,0], xyz_traj[-1,1], xyz_traj[-1,2], c='b' , marker='o')
plt.show()

T_tool = SE3.Trans(0.25,0,0.0)*SE3.Trans(xyz_traj)*SE3.OA([0,-1,0],[1,0,0])
we = [1,1,1,0,0,0]
sol = cobot.ikine_LM(T_tool, q0=[0,np.deg2rad(90),0,0,0,0], ilimit=100, slimit=100, tol=1e-6 , mask = we)
print(sol)
cobot.plot(q=sol.q, limits=[-0.3,0.6,-0.6,0.6,-0.1,1],
backend='pyplot', shadow = True, jointaxes = True, eeframe = True, block = True)


#individual inverse kinematics solutions
# Create a list to hold the IK solutions
ik_solutions = []

# Loop over each via point (each row in via corresponds to a point)
for punto in via:
    # Create the tool transformation for the current point
    T_tool_pt = SE3.Trans(0.25, 0, 0.0) * SE3.Trans(punto) * SE3.OA([0, -1, 0], [1, 0, 0])
    
    # Compute the inverse kinematics for the current point
    sol = cobot.ikine_LM(T_tool_pt,
                         q0=[0, np.deg2rad(90), 0, 0, 0, 0],
                         ilimit=100, slimit=100, tol=1e-6,
                         mask=we)
    
    # Save the solution if needed
    ik_solutions.append(sol.q)
    
    # Plot the current configuration (each plot is separate, no trajectory)
    cobot.plot(q=sol.q,
               limits=[-0.3, 0.6, -0.6, 0.6, -0.1, 1],
               backend='pyplot',
               shadow=True,
               jointaxes=True,
               eeframe=True,
               block=False)
    plt.pause(0.5)  # Pause briefly to view each configuration

# Keep the final plot window open
plt.show()