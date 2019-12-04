from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix


from numpy import inf, cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *

class Unicycle(KinematicModel):
    """
    This is the unicycle model, the robot can only turn around and move forward and backward.
    """

    def __init__(self, agent, dT, auto=True, init_state = [-5,-5,0,0]):
        self.max_vr = 2
        self.max_ar = 4
        self.max_vt = pi
        self.k_v = 2
        self.k_theta = 2
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
        self.x[2,0] = np.linalg.norm(v)
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
    #     x[2,0] = np.linalg.norm(p[[2,3],0]);
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
        dis = np.linalg.norm(dp);
        v = self.observe(self.get_V())[[0,1]];
        
        theta_R = self.observe(self.x[3,0])

        if(np.linalg.norm(v) < 1e-5):
            v[0] = 1e-5 * cos(theta_R)
            v[1] = 1e-5 * sin(theta_R)
        
        if dis < 1e-5:
            dis = 1e-5
        
        d_theta = arctan2(dp[1], dp[0]) - theta_R

        if abs(d_theta) > pi:
            while d_theta > pi:
                d_theta = d_theta - 2 * pi;
            while d_theta < -pi:
                d_theta = d_theta + 2 * pi;

        # if d_theta > pi/2 :
        #     d_theta = d_theta % (pi/2)

        u0 = matrix(zeros((2,1)));
        
        sgn = 1
        if np.asscalar(dp.T * v) < 0:
            sgn = -1
        
        u0[0,0] = dp[0] * cos(theta_R) + dp[1] * sin(theta_R) - self.k_v * np.linalg.norm(v) * sgn;
        u0[1,0] = self.k_theta * d_theta
        
        return u0;

    def load_R2D2(self, pos, color, scale, render_node):
        ret = loader.loadModel("resource/R2D2/R2D2.obj")
        ret.reparentTo(render_node)
        ret.setColor(color[0], color[1], color[2], color[3]);
        ret.setScale(scale / 30)
        ret.setPos(pos[0], pos[1], pos[2]-0.3)
        ret.setP(90)
        ret.setR(90)

        # tacc = loader.loadTexture('resource/R2D2/texture accesoires.jpg')
        # ret.find('**/*').setTexture(tacc, 1)

        print(ret.get_children())

        ttete = loader.loadTexture('resource/R2D2/texture tete.jpg')
        ret.find('**/tete').setTexture(ttete, 1)

        tcorps = loader.loadTexture('resource/R2D2/texture corps.jpg')
        ret.find('**/corps').setTexture(tcorps, 1)

        # taxe = loader.loadTexture('resource/R2D2/texture bras.jpg')
        # ret.find('**/axes_bras*').setTexture(taxe, 1)

        pivot = render_node.attachNewNode("unicycle")
        pivot.setPos(pos[0], pos[1], pos[2]) # Set location of pivot point
        ret.wrtReparentTo(pivot) # Preserve absolute position

        return pivot;

    def load_unicycle(self, loader, color, scale, render_node):
            
        pos = list(self.get_P()[:,0])
        ret = loader.loadModel("resource/cube")
        ret.reparentTo(render_node)
        ret.setColor(color[0], color[1], color[2], color[3]);
        ret.setScale(scale,0.1,scale);
        ret.setPos(pos[0], pos[1], pos[2]);
        return ret

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        
        KinematicModel.load_model(self, render, loader, color, scale)
        pos = list(self.get_P()[:,0])
        # self.agent_model = self.load_R2D2(pos, color, scale, render.attachNewNode('agent'))
        self.agent_model = self.load_unicycle(loader, color, scale, render.attachNewNode('agent'))
        self.goal_model = self.add_sphere([self.goal[0], self.goal[1],0], color[:-1]+[0.5], scale, render.attachNewNode('goal'))

        
    def redraw_model(self):
        self.agent_model.setPos(self.get_P()[0], self.get_P()[1], 0);
        self.agent_model.setH(self.x[3,0] / np.pi * 180);
        self.goal_model.setPos(self.goal[0], self.goal[1], 0)

    def model_auxiliary(self):
        if not hasattr(self, 'robot_v'):
            [self.robot_v, self.robot_u] = self.draw_movement(list(self.get_PV()[:,0]), [0,0,0]);
        else:
            self.move_seg(self.robot_v, list(self.get_P()[:,0]), list(self.get_V()[:,0]))
