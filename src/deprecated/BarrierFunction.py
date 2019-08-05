from .MobileAgent import MobileAgent
import numpy as np
from cvxopt import solvers, matrix
from numpy.matlib import repmat
from numpy import zeros, eye, ones, sqrt, asscalar, log


class BarrierFunction(MobileAgent):

    def __init__(self):

        self.t = 0.5
        self.gamma = 2
        self.half_plane_ABC = []
        self.d_min = 2
        
        MobileAgent.__init__(self);
        self.safe_set = [0,0,0]

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):
        
        dim = np.shape(Mr)[0] // 2
        p_idx = np.arange(dim)
        v_idx = p_idx + dim

        d = np.linalg.norm(Mr[p_idx] - Mh[p_idx])

        dot_Mr = p_Mr_p_Xr * dot_Xr
        dot_Mh = p_Mh_p_Xh * dot_Xh

        dM = Mr - Mh
        dot_dM = dot_Mr - dot_Mh
        dp = dM[p_idx,0]
        dv = dM[v_idx,0]

        dot_dp = dot_dM[p_idx,0]
        dot_dv = dot_dM[v_idx,0]

        #dot_d is the component of velocity lies in the dp direction
        dot_d =  asscalar(dp.T * dv / d)

        d = 1e-3 if d == 0 else d
        dot_d = 1e-3 if dot_d == 0 else dot_d

        p_d_p_Mr = np.vstack([ dp / d, zeros((dim,1))])
        p_d_p_Xr = p_Mr_p_Xr.T * p_d_p_Mr

        
        p_dot_d_p_dp = dv / d - asscalar(dp.T * dv) * dp / (d**3)
        p_dot_d_p_dv = dp / d
        
        p_dp_p_Mr = np.hstack([eye(dim), zeros((dim,dim))])

        p_dv_p_Mr = np.hstack([zeros((dim,dim)), eye(dim)])

        
        p_dot_d_p_Mr = p_dp_p_Mr.T * p_dot_d_p_dp + p_dv_p_Mr.T * p_dot_d_p_dv

    
        h = (d ** 2 + dot_d * self.t - self.d_min)
        h = 1e-100 if h < 0 else h
        p_h_p_Mr = (2 * d * p_d_p_Mr + p_dot_d_p_Mr * self.t)
        p_h_p_Mh = -p_h_p_Mr
        
  
        B = -1*log(h / (1+h))
        p_B_p_h = (-1 / (h+1) / h)

        p_B_p_Xr = p_B_p_h * p_Mr_p_Xr.T * p_h_p_Mr
        p_B_p_Xh = p_B_p_h * p_Mh_p_Xh.T * p_h_p_Mh
        LfB = p_B_p_Xr.T * fx
        LgB = p_B_p_Xr.T * fu
        A = matrix(LgB)
        b = matrix(self.gamma / B - LfB - p_B_p_Xh.T * dot_Xh)

        A = A / abs(b)
        b = b / abs(b)

        Q = matrix(eye(np.shape(u0)[0]))
        p = matrix(- 2 * u0)

        nu = np.shape(u0)[0]
        G = matrix(np.vstack([eye(nu), -eye(nu)]))
        r = matrix(np.vstack([max_u, -min_u]))

        A = matrix([[A,G]])
        b = matrix([[b,r]])

        u = u0
        self.fuck = False
        try:
            solvers.options['feastol']=1e-9
            solvers.options['show_progress'] = False
            sol=solvers.qp(Q, p, A, b)
            u = np.vstack(sol['x'])
        except:
            pass
            
        
        self.half_plane_ABC = matrix([[A],[-b - A[:,0]*Mr[0,0] - A[:,1]*Mr[1,0]]])
        self.ABC = matrix([[A],[-b]])
        
        return u