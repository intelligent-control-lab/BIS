from .MobileAgent import MobileAgent
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, sqrt, asscalar

from cvxopt import solvers, matrix

class Naive(MobileAgent):

    d_min = 2
    half_plane_ABC = []

    def __init__(self):
        
        MobileAgent.__init__(self);
        self.safe_set = [0,0,0]

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):
               
        dim = np.shape(Mr)[0] // 2
        p_idx = np.arange(dim)
        v_idx = p_idx + dim

        d = np.linalg.norm(Mr[p_idx] - Mh[p_idx])
        
        # sgn = -1 if np.asscalar((Mr[[0,1],0] - Mh[[0,1],0]).T * (Mr[[2,3],0] - Mh[[2,3],0])) < 0 else 1
        # dot_d = sgn * sqrt((Mr[2,0] - Mh[2,0])**2 + (Mr[3,0] - Mh[3,0])**2)

        dM = Mr - Mh
        dp = dM[p_idx,0]

        d = 1e-3 if d == 0 else d

        p_d_p_Mr = np.vstack([ dp / d, zeros((dim,1))])
        p_d_p_Mh = np.vstack([-dp / d, zeros((dim,1))])

        p_d_p_Xr = p_Mr_p_Xr.T * p_d_p_Mr
        p_d_p_Xh = p_Mh_p_Xh.T * p_d_p_Mh

        if d > self.d_min:
            return u0

        u = u0

        return u
        
        
    



