import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy.random import rand, randn
from numpy.linalg import norm, inv
from numpy import cos, sin, arccos, sqrt
from models import *

class MobileAgent:

    max_a = 4 # max acceleration per second

    def __init__(self):
        pass

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):
        
        return u0;

     