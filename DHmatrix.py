import sympy as sp
from sympy.matrices import rot_axis3
#para graficar
import matplotlib.pyplot as plt
import numpy as np
#generar matriz DH
from spatialmath import *
from spatialmath.base import *

theta, d , a , alpha = sp.symbols('theta , d ,a, alpha')

TDH = trotz(theta) @ transl(0,0,d) @ transl(a,0,0) @ trotx(alpha)
sp.pprint(TDH)
print(type(TDH))

T = sp.Matrix(TDH)


theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6 =sp.symbols('theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6')
#De la tabla DH
T01 = T.subs({d :0.680, a: 0.200, alpha: -sp.pi/2})
T01 = T01.subs({theta: theta_1})
sp.pprint(T01)
