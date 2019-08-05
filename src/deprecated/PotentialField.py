from .MobileAgent import MobileAgent
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix, sqrt, asscalar


class PotentialField(MobileAgent):

    d_min = 2 # distance to trigger static potential field
    lambd = 5 # strength of the entire field
    beta = 2
    eta = 3

    def __init__(self):

        MobileAgent.__init__(self)

    def phi(self, x, v):
        # size(v.T)
        # size(x)

        nv = np.linalg.norm(v)
        nx = np.linalg.norm(x)
        nv = 1e-3 if nv == 0 else nv
        nx = 1e-3 if nx == 0 else nx

        cosin = asscalar(v.T * x) / (nv * nx)

        # dcosin_dx = [nv**2 * x[1,0] * (v[0,0] * x[1,0] - v[1,0] * x[0,0]) / (nv**3 * nx**3),
        #             nv**2 * x[0,0] * (v[1,0] * x[0,0] - v[0,0] * x[1,0]) / (nv**3 * nx**3) ]
        # dnx_dx = [x[0,0] / nx, x[1,0] / nx]
        d_cos_d_x = v / (nv * nx) - asscalar(v.T * x) * x / (nv * nx**3)
        d_nx_d_x = x / nx
        # print('x')
        # print(x)
        # print('v')
        # print(v)
        # print('cosin')
        # print(cosin)

        ret = matrix(zeros(np.shape(v)))
        if nx < self.d_min:
            static_field = -self.lambd * (1 / nx - 1 / self.d_min) * (-1/nx**2) * d_nx_d_x
            # print('static')
            # print(static_field)
            ret = ret + static_field

        if cosin < 0:
            ret = ret + self.lambd * (-cosin)**(self.beta-1) * nv / nx * ( self.beta * d_cos_d_x - cosin / nx * d_nx_d_x)
            # print('dynamic')
            # print(self.lambd * (-cosin)**(self.beta-1) * nv / nx * ( self.beta * d_cos_d_x - cosin / nx * d_nx_d_x))
        
        # ret 
        # only suitable for beta = 2. The equation below is com_putationally equivalent to the above one, but more stable when it.Ts close to 0.
        # ret = - self.lambd * cosin / nx**4 * np.matrix([[3*v[1,0]*x[0,0]*x[1,0] + v[0,0] * (x[0,0]**2 - 2*x[1,0]**2)],
                                                        #  [3*v[0,0]*x[0,0]*x[1,0] + v[1,0] * (x[1,0]**2 - 2*x[0,0]**2)]])

        return ret


    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):

        m_dim = np.shape(Mr)[0] // 2
        m_p_idx = np.arange(m_dim)
        m_v_idx = m_p_idx + m_dim

        x_dim = np.shape(Xr)[0] // 2
        x_p_idx = np.arange(x_dim)
        x_v_idx = x_p_idx + x_dim

        d = np.linalg.norm(Mr[m_p_idx] - Mh[m_p_idx])
        
        # sgn = -1 if np.asscalar((Mr[[0,1],0] - Mh[[0,1],0]).T * (Mr[[2,3],0] - Mh[[2,3],0])) < 0 else 1
        # dot_d = sgn * sqrt((Mr[1,0] - Mh[1,0])**2 + (Mr[3,0] - Mh[3,0])**2)

        dot_Mr = p_Mr_p_Xr * dot_Xr
        dot_Mh = p_Mh_p_Xh * dot_Xh

        dM = Mr - Mh
        dot_dM = dot_Mr - dot_Mh
        dp = dM[m_p_idx,0]
        dv = dM[m_v_idx,0]

        # print('================')
        # print('Mr')
        # print(Mr)
        u_Mr = self.phi(dp, dv)
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
        
        u_Xr = fu.T * p_Mr_p_Xr.T * np.vstack([np.zeros((m_dim,1)),u_Mr])
        # print('u_Xr')
        # print(u_Xr)

        u = u0 + u_Xr
        
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