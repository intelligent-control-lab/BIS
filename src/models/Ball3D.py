from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix


from numpy import cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *

class Ball3D(KinematicModel):

    """
    This the 3D ball model. The robot can move to any direction.
    """

    def __init__(self, agent, dT, auto=True, init_state = [-5,-5,0,0,0,0]):
        self.max_v = 2
        self.max_a = 4
        KinematicModel.__init__(self, init_state, agent, dT, auto, is_2D = False)

    def init_x(self, init_state):
        x = np.vstack(init_state)
        self.x = matrix(zeros((6,1)))
        self.set_P(x[[0,1,2]])
        self.set_V(x[[3,4,5]])
        
        self.u = matrix(zeros((3,1)))

    def set_saturation(self):
        self.max_u =  matrix([[self.max_a], [self.max_a], [self.max_a]])
        self.min_u = -matrix([[self.max_a], [self.max_a], [self.max_a]])
        self.max_x =  matrix([np.inf, np.inf, np.inf, self.max_v, self.max_v, self.max_v]).T
        self.min_x = -matrix([np.inf, np.inf, np.inf, self.max_v, self.max_v, self.max_v]).T

    def estimate_state(self, obstacle):
        self.kalman_estimate_state()
 
    def get_P(self):
        return self.x[[0,1,2]];
    
    def get_V(self):
        return self.x[[3,4,5]];


    def get_closest_X(self, Xh):
        self.m = self.get_PV()
        return self.m

    def set_P(self, p):
        self.x[0,0] = p[0];
        self.x[1,0] = p[1];
        self.x[2,0] = p[2];

    def set_V(self, v):
        self.x[3,0] = v[0];
        self.x[4,0] = v[1];
        self.x[5,0] = v[2];

    def A(self):
        A = matrix(eye(6,6));
        A[0,3] = self.dT;
        A[1,4] = self.dT;
        A[2,5] = self.dT;
        return A;

    def B(self):
        B = matrix(zeros((6,3)));
        B[0,0] = self.dT**2/2;
        B[1,1] = self.dT**2/2;
        B[2,2] = self.dT**2/2;
        B[3,0] = self.dT;
        B[4,1] = self.dT;
        B[5,2] = self.dT;
        return B;

    def p_M_p_X(self): # p closest point p X
        ret = matrix(eye(6))
        return ret

    def u_ref(self):
        K = matrix([[1,0,0,2,0,0],[0,1,0,0,2,0],[0,0,1,0,0,2]]);
        dp = self.observe(self.goal[[0,1,2]] - self.m[[0,1,2]]);
        dis = np.linalg.norm(dp);
        v = self.observe(self.m[[3,4,5]]);

        if dis > 2:
            u0 = dp / max(abs(dp)) * self.max_a;
        else:
            u0 = - K*(self.m - self.goal);
        return u0;






    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        KinematicModel.load_model(self, render, loader, color, scale)
        self.agent_model = self.add_sphere(list(self.get_P()[:,0]), color, scale, render.attachNewNode('agent'))
        self.goal_model = self.add_sphere([self.goal[0], self.goal[1], self.goal[2]], color[:-1]+[0.5], scale, render.attachNewNode('goal'))

    def redraw_model(self):
        p = self.get_P()
        self.agent_model.setPos(p[0], p[1] , p[2])
        self.goal_model.setPos(self.goal[0], self.goal[1], self.goal[2])
    
    def model_auxiliary(self):
        if not hasattr(self, 'robot_v'):
            [self.robot_v, self.robot_u] = self.draw_movement(list(self.get_PV()[:,0]), list(self.u[:,0])+[0]);
        else:
            self.move_seg(self.robot_v, list(self.get_P()[:,0]), list(self.get_V()[:,0]))
            self.move_seg(self.robot_u, list(self.get_P()[:,0]), list(self.u[:,0])+[0])
            self.remove_half_planes()
            self.add_half_planes()

    def draw_half_plane(self, coef):
        segs = LineSegs( )
        segs.setThickness( 20.0 )
        segs.setColor( Vec4(0.1, 0.5, 0.8, 0.5) )
        A = coef[0];
        B = coef[1];
        C = coef[2];
        x0 = -10;
        x1 = 10;
        if B == 0:
            if A == 0:
                x0 = -100;
                x1 = -100;
            else:
                x0 = -C / A;
                x1 = -C / A;
            y0 = -10;
            y1 = 10;
        else:
            y0 = (-C - A * x0) / B;
            y1 = (-C - A * x1) / B;
        p_from = LVector3f(x0, y0, 0)
        p_to = LVector3f(x1, y1, 0)
        segs.moveTo( p_from )
        segs.drawTo( p_to )
        half_plane = segs.create( )
        return half_plane

    def add_half_planes(self):
        ABCs = self.agent.half_plane_ABC
        self.half_plane = []
        self.half_plane_handle = []
        self.valid_half = np.ones((2*self.map_size+1,2*self.map_size+1))
        for i in range(np.shape(ABCs)[0]):
            coef = np.squeeze(np.asarray(ABCs[i,:]))
            self.half_plane.append(self.draw_half_plane(coef));
            self.half_plane_handle.append(self.render.attachNewNode(self.half_plane[i]));
            self.test_half(self.half_plane_handle[i], coef)
        self.draw_valid_half()

    def remove_half_planes(self):
        if not hasattr(self, 'half_plane_handle'):
            return
        ABCs = self.agent.half_plane_ABC
        for i in range(np.shape(ABCs)[0]):
            self.half_plane_handle[i].removeNode()


    def test_half(self, parent, coef):
        A = coef[0];
        B = coef[1];
        C = coef[2];
        m = self.map_size
        for i in range(-m,m+1):
            for j in range(-m,m+1):
                if A*i+B*j+C >= 0:
                    self.valid_half[i+m][j+m] = False

    def draw_valid_half(self):
        m = self.map_size
        for i in range(-m,m+1):
            for j in range(-m,m+1):
                if self.valid_half[i+m][j+m]:
                    Ball3D = loader.loadModel("resource/planet_sphere")
                    Ball3D.setTransparency(TransparencyAttrib.MAlpha);
                    Ball3D.setColor(0.1, 0.9, 0.1, 0.3);
                    Ball3D.setScale(0.1);
                    Ball3D.setPos(i,j,0);
                    Ball3D.reparentTo(self.half_plane_handle[0])
