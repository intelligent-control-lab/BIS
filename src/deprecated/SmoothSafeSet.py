from .MobileAgent import MobileAgent
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix, sqrt, asscalar


class SmoothSafeSet(MobileAgent):

    D = 2 # safety index, d_min^2 + lamda*dT + eta*dT
    k_v = 2 # factor for punish relative velocity
    d_min = 2
    yita = 10
    lambd = 0.5
    alpha = 1
    dstart = d_min + 2;

    def __init__(self):
        
        MobileAgent.__init__(self);
        self.safe_set = [0,0,0]

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):
        # print(self.x_est)
        d = sqrt((Mr[0,0] - Mh[0,0])**2 + (Mr[1,0] - Mh[1,0])**2)
        
        # sgn = -1 if np.asscalar((Mr[[0,1],0] - Mh[[0,1],0]).T * (Mr[[2,3],0] - Mh[[2,3],0])) < 0 else 1
        # dot_d = sgn * sqrt((Mr[2,0] - Mh[2,0])**2 + (Mr[3,0] - Mh[3,0])**2)

        dot_Mr = p_Mr_p_Xr * dot_Xr
        dot_Mh = p_Mh_p_Xh * dot_Xh

        dM = Mr - Mh
        dot_dM = dot_Mr - dot_Mh
        dp = dM[[0,1],0]
        dv = dM[[2,3],0]

        dot_dp = dot_dM[[0,1],0]
        dot_dv = dot_dM[[2,3],0]

        #dot_d is the component of velocity lies in the dp direction
        dot_d = dp.T * dv / d

        p_dot_d_p_dp = dv / d - asscalar(dp.T * dv) * dp / (d**3)
        p_dot_d_p_dv = dp / d
        
        p_dp_p_Mr = zeros((2,4))
        p_dp_p_Mr[0,0] = 1
        p_dp_p_Mr[1,1] = 1
        p_dp_p_Mh = -p_dp_p_Mr

        p_dv_p_Mr = zeros((2,4))
        p_dv_p_Mr[0,2] = 1
        p_dv_p_Mr[0,3] = 1
        p_dv_p_Mh = -p_dv_p_Mr

        p_dot_d_p_Mr = p_dp_p_Mr.T * p_dot_d_p_dp + p_dv_p_Mr.T * p_dot_d_p_dv
        p_dot_d_p_Mh = p_dp_p_Mh.T * p_dot_d_p_dp + p_dv_p_Mh.T * p_dot_d_p_dv

        p_dot_d_p_Xr = p_Mr_p_Xr.T * p_dot_d_p_Mr
        p_dot_d_p_Xh = p_Mh_p_Xh.T * p_dot_d_p_Mh


        d = 1e-3 if d == 0 else d
        dot_d = 1e-3 if dot_d == 0 else dot_d


        p_d_p_Mr = np.matrix([(Mr[0,0] - Mh[0,0]) / d, (Mr[1,0] - Mh[1,0]) / d, 0, 0]).T
        p_d_p_Mh = np.matrix([(Mh[0,0] - Mr[0,0]) / d, (Mh[1,0] - Mr[1,0]) / d, 0, 0]).T

        p_d_p_Xr = p_Mr_p_Xr.T * p_d_p_Mr
        p_d_p_Xh = p_Mh_p_Xh.T * p_d_p_Mh
        

        phi = self.d_min**2 + self.yita * dT + self.lambd * dT - d**2 - self.k_v * dot_d;

        p_phi_p_Xr = - 2 * d * p_d_p_Xr - self.k_v * p_dot_d_p_Xr;
        p_phi_p_Xh = - 2 * d * p_d_p_Xh - self.k_v * p_dot_d_p_Xh;

        dot_phi = p_phi_p_Xr.T * dot_Xr + p_phi_p_Xh.T * dot_Xh;


        L = p_phi_p_Xr.T * fu;
        S = - self.yita - self.lambd - p_phi_p_Xh.T * dot_Xh - p_phi_p_Xr.T * fx;

        # print('-=-=-=-=-=')
        if phi <= 0 or asscalar(L * u0) < asscalar(S):
            u_opt = u0;
        else:
            u_opt = u0 - (asscalar(L * u0 - S) * L.T / asscalar(L * L.T));

        sm_lambda = (1 + np.exp(-self.alpha)) / (1 + np.exp(self.alpha * (d - self.dstart)*(2/self.d_min) - 1));
    
        u = sm_lambda * u_opt + (1 - sm_lambda) * u0;
        
 
        return u