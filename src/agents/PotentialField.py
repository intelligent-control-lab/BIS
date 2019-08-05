from .MobileAgent import MobileAgent
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix, sqrt, asscalar


class PotentialField(MobileAgent):
    """
    Potential field method. This method is inspired from gravity potential field.
    """
    def __init__(self):
        MobileAgent.__init__(self)
        self.d_min = 2
        self.yita = 10
        self.lambd = 0.5
        self.k_v = 1
        self.c1 = 3

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):


        dim = np.shape(Mr)[0] // 2
        p_idx = np.arange(dim)
        v_idx = p_idx + dim

        d = np.linalg.norm(Mr[p_idx] - Mh[p_idx])
        
        # sgn = -1 if np.asscalar((Mr[[0,1],0] - Mh[[0,1],0]).T * (Mr[[2,3],0] - Mh[[2,3],0])) < 0 else 1
        # dot_d = sgn * sqrt((Mr[2,0] - Mh[2,0])**2 + (Mr[3,0] - Mh[3,0])**2)

        dot_Mr = p_Mr_p_Xr * dot_Xr
        dot_Mh = p_Mh_p_Xh * dot_Xh

        dM = Mr - Mh
        dot_dM = dot_Mr - dot_Mh
        dp = dM[p_idx,0]
        dv = dM[v_idx,0]

        dot_dp = dot_dM[p_idx,0]
        dot_dv = dot_dM[v_idx,0]

        #dot_d is the component of velocity lies in the dp direction
        dot_d = dp.T * dv / d

        p_dot_d_p_dp = dv / d - asscalar(dp.T * dv) * dp / (d**3)
        p_dot_d_p_dv = dp / d
        
        p_dp_p_Mr = np.hstack([eye(dim), zeros((dim,dim))])
        p_dp_p_Mh = -p_dp_p_Mr

        p_dv_p_Mr = np.hstack([zeros((dim,dim)), eye(dim)])
        p_dv_p_Mh = -p_dv_p_Mr

        p_dot_d_p_Mr = p_dp_p_Mr.T * p_dot_d_p_dp + p_dv_p_Mr.T * p_dot_d_p_dv
        p_dot_d_p_Mh = p_dp_p_Mh.T * p_dot_d_p_dp + p_dv_p_Mh.T * p_dot_d_p_dv

        p_dot_d_p_Xr = p_Mr_p_Xr.T * p_dot_d_p_Mr
        p_dot_d_p_Xh = p_Mh_p_Xh.T * p_dot_d_p_Mh


        d = 1e-3 if d == 0 else d
        dot_d = 1e-3 if dot_d == 0 else dot_d

        p_d_p_Mr = np.vstack([ dp / d, zeros((dim,1))])
        p_d_p_Mh = np.vstack([-dp / d, zeros((dim,1))])

        p_d_p_Xr = p_Mr_p_Xr.T * p_d_p_Mr
        p_d_p_Xh = p_Mh_p_Xh.T * p_d_p_Mh
        

        # print('================')
        # print('Mr')
        # print(Mr)
        phi = self.d_min**2 + self.lambd * dT - d**2 - self.k_v * dot_d;
        p_dot_d_p_Mr = p_dp_p_Mr.T * p_dot_d_p_dp + p_dv_p_Mr.T * p_dot_d_p_dv
        p_d_p_Mr = np.vstack([ dp / d, zeros((dim,1))])
        p_phi_p_Mr = - 2 * d * p_d_p_Mr - self.k_v * p_dot_d_p_Mr
        
        u_Mr = -self.c1 * p_phi_p_Mr
        # print('===============')
        # print('u0')
        # print(u0)
        # print('u_Mr')
        # print(u_Mr)
        # u_Mr = self.phi(dp, dv)
        # print('m_v_idx, x_v_idx')
        # print(m_v_idx, x_v_idx)
        # print('p_Mr_p_Xr[m_v_idx, :].T')
        # print(p_Mr_p_Xr[m_v_idx, :].T)
        # print('fu.T')
        # print(fu.T)
        # print('p_Mr_p_Xr.T')
        # print(p_Mr_p_Xr.T)
        # print('p_Mr_p_Xr.T')
        # print(p_Mr_p_Xr.T)
        # print('np.vstack([np.zeros(m_dim),u_Mr])')
        # print(np.vstack([np.zeros((m_dim,1)),u_Mr]))
        
        u_Xr = fu.T * p_Mr_p_Xr.T * u_Mr
        # print('u_Xr')
        # print(u_Xr)
        u = u0
        if phi > 0:
            u = u + u_Xr
        

        
        self.fuck = False
        # print('u0')
        # print(u0)
        # print('u_Xr')
        # print(u_Xr)
        # print('u')
        # print(u)
        self.half_plane_ABC = []
        # print('u')
        # print(u)
        # print('u0')
        # print(u0)
        
        
        return u