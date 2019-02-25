from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy.random import rand, randn
from numpy.linalg import norm, inv
from numpy import inf, cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *

class Unicycle(KinematicModel):

    max_vr = 2

    max_ar = 4
    max_vt = pi

    k_v = 2
    k_theta = 2

    def __init__(self, agent, dT, auto=True, init_state = [-5,-5,0,0]):
        KinematicModel.__init__(self, init_state, agent, dT, auto, is_2D = True);

    def set_saturation(self):
        self.max_u =  matrix([[self.max_ar], [self.max_vt]]);
        self.min_u = -matrix([[self.max_ar], [self.max_vt]]);
        self.max_x =  matrix([inf, inf, self.max_vr, inf]).T;
        self.min_x = -matrix([inf, inf, self.max_vr, inf]).T;
    
    #TODO: init_state shoubld be a param class. the same as agent params.
    def init_x(self, init_state):
        x = np.vstack(init_state)
        self.x = matrix(zeros((4,1)))
        self.set_P(x[[0,1]])
        self.set_V(x[[2,3]])
        self.u = matrix(zeros((2,1)))

    def get_P(self):
        return np.vstack([self.x[[0,1]],0])
    
    def get_V(self):
        return np.vstack([self.x[2,0] * cos(self.x[3,0]), self.x[2,0] * sin(self.x[3,0]), 0])

    def set_P(self, p):
        self.x[0,0] = p[0]
        self.x[1,0] = p[1]

    def set_V(self, v):
        self.x[2,0] = norm(v)
        self.x[3,0] = arctan2(v[1], v[0])
    
    def get_closest_X(self, Xh):
        self.m = self.get_PV()
        return self.m
        
    def A(self):
        A = eye(4,4);
        A[0,2] = cos(self.x[3,0]) * self.dT;
        A[1,2] = sin(self.x[3,0]) * self.dT;
        return A;

    def B(self):
        B = matrix(zeros((4,2)));
        B[2,0] = self.dT;
        B[3,1] = self.dT;
        return B;

    def filt_u(self, u):
        u = np.minimum(u,  self.max_u);
        u = np.maximum(u, -self.max_u);
        return u;

    def filt_x(self, x):
        while x[3] > pi:
            x[3] = x[3] - 2 * pi;
        while x[3] < -pi:
            x[3] = x[3] + 2 * pi;
        x = np.minimum(x,  self.max_x);
        x = np.maximum(x,  self.min_x);
        return x
        
    # def inv_J(self, p):
    #     x = matrix(zeros((4,1)));
    #     x[0,0] = p[0,0];
    #     x[1,0] = p[1,0];
    #     x[2,0] = norm(p[[2,3],0]);
    #     x[3,0] = np.arctan2(p[3,0], p[2,0]);
    #     return x;

    def p_M_p_X(self): # p closest point p X
        ret = matrix(zeros((6,4)));
        ret[0,0] = 1
        ret[1,1] = 1
        ret[3,2] = cos(self.x[3,0])
        ret[3,3] = -self.x[2,0] * sin(self.x[3,0])
        ret[4,2] = sin(self.x[3,0])
        ret[4,3] = self.x[2,0] * cos(self.x[3,0])
        return ret

    def u_ref(self):
        
        dp = self.observe((self.goal - self.get_PV())[[0,1]]);
        dis = norm(dp);
        v = self.observe(self.get_V())[[0,1]];
        
        theta_R = self.observe(self.x[3,0])

        if(norm(v) < 1e-5):
            v[0] = 1e-5 * cos(theta_R)
            v[1] = 1e-5 * sin(theta_R)
        
        if dis < 1e-5:
            dis = 1e-5
        
        d_theta = arctan2(dp[1], dp[0]) - theta_R

        # if abs(d_theta) > pi:
        #     d_theta = d_theta % pi

        # if d_theta > pi/2 :
        #     d_theta = d_theta % (pi/2)

        u0 = matrix(zeros((2,1)));
        
        sgn = 1
        if np.asscalar(dp.T * v) < 0:
            sgn = -1
        
        u0[0,0] = dp[0] * cos(theta_R) + dp[1] * sin(theta_R) - self.k_v * norm(v) * sgn;
        u0[1,0] = self.k_theta * d_theta
        
        return u0;

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 0.8], scale=0.5):
        
        KinematicModel.load_model(self, render, loader, color, scale)

        pos = list(self.get_P()[:,0])
        self.robot_sphere = loader.loadModel("resource/cube")
        self.robot_sphere.reparentTo(render)
        self.robot_sphere.setColor(color[0], color[1], color[2], color[3]);
        self.robot_sphere.setScale(scale,0.1,scale);
        self.robot_sphere.setPos(pos[0], pos[1], pos[2]);
        self.robot_goal_sphere = self.add_sphere([self.goal[0], self.goal[1],0], color[:-1]+[0.5], scale);

        
    def redraw_model(self):
        self.robot_sphere.setPos(self.get_P()[0], self.get_P()[1], 0);
        self.robot_sphere.setH(self.x[3,0] / np.pi * 180);
        self.robot_goal_sphere.setPos(self.goal[0], self.goal[1], 0)

    def model_auxiliary(self):
        if not hasattr(self, 'robot_v'):
            [self.robot_v, self.robot_u] = self.draw_movement(list(self.get_PV()[:,0]), [0,0,0]);
        else:
            self.move_seg(self.robot_v, list(self.get_P()[:,0]), list(self.get_V()[:,0]))
