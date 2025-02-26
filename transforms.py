import numpy as np
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *
from math import pi

T0 = rotz(0, unit ='deg')
trplot( T0 ,dims=[-1,1,-1,1,-1,1], color='k')

TA = rotz(90, unit ='deg')
trplot( TA ,dims=[-1,1,-1,1,-1,1], color='g')

P = np.array([0,0,0])
ax = plt.gca()
ax.scatter(P[0], P[1], P[2], color='r',label = 'P')

plt.gca().view_init(elev=25, azim=44)

plt.show()