from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix


from numpy import cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *

class Ball(KinematicModel):

    """
    This the 2D ball model. The robot can move to any direction on the plane.
    """
    
    def __init__(self, agent, dT, auto=True, init_state = [-5,-5,0,0]):

        self.max_v = 2
        self.max_a = 4

        KinematicModel.__init__(self, init_state, agent, dT, auto, is_2D = True)
        self.goals[2,:] = zeros(100)
        self.goal = np.vstack(self.goals[:,0])

    def init_x(self, init_state):
        x = np.vstack(init_state)
        self.x = matrix(zeros((4,1)))
        self.set_P(x[[0,1]])
        self.set_V(x[[2,3]])
        
        self.u = matrix(zeros((2,1)))
        self.m = self.get_PV()

    def set_saturation(self):
        self.max_u =  matrix([[self.max_a], [self.max_a]])
        self.min_u = -matrix([[self.max_a], [self.max_a]])
        self.max_x =  matrix([np.inf, np.inf, self.max_v, self.max_v]).T
        self.min_x = -matrix([np.inf, np.inf, self.max_v, self.max_v]).T

    def estimate_state(self, obstacle):
        self.kalman_estimate_state()
 
    def get_P(self):
        return np.vstack([self.x[[0,1]], 0]);
    
    def get_V(self):
        return np.vstack([self.x[[2,3]], 0]);

    def get_PV(self):
        return np.vstack([self.get_P(), self.get_V()])

    def get_closest_X(self, Xh):
        self.m = self.get_PV()
        return self.m

    def set_P(self, p):
        self.x[0,0] = p[0];
        self.x[1,0] = p[1];

    def set_V(self, v):
        self.x[2,0] = v[0];
        self.x[3,0] = v[1];

    def A(self):
        A = matrix(eye(4,4));
        A[0,2] = self.dT;
        A[1,3] = self.dT;
        return A;

    def B(self):
        B = matrix(zeros((4,2)));
        B[0,0] = self.dT**2/2;
        B[1,1] = self.dT**2/2;
        B[2,0] = self.dT;
        B[3,1] = self.dT;
        return B;

    def p_M_p_X(self): # p closest point p X
        ret = zeros((6,4))
        ret[0,0] = 1
        ret[1,1] = 1
        ret[3,2] = 1
        ret[4,3] = 1
        return ret

    def u_ref(self):
        K = matrix([[1,0,2,0],[0,1,0,2]]);
        dp = self.observe(self.goal[[0,1]] - self.m[[0,1]]);
        dis = np.linalg.norm(dp);
        v = self.observe(self.m[[3,4]]);

        if dis > 2:
            u0 = dp / max(abs(dp)) * self.max_a;
        else:
            u0 = - K*(self.m[[0,1,3,4]] - self.goal[[0,1,3,4]]);
        return u0;



    def add_BB8(self, pos, color, scale, render_node):
        ret = loader.loadModel("resource/BB8/bb8.obj")
        ret.reparentTo(render_node)
        ret.setTransparency(TransparencyAttrib.MAlpha)
        # ret.setColor(color[0], color[1], color[2], color[3])
        ret.setP(90)
        ret.setScale(scale / 50)
        ret.setPos(pos[0], pos[1], pos[2]-0.5)
        
        body_diff = loader.loadTexture('resource/BB8/Body diff MAP.jpg')
        tbd = TextureStage('body diff')
        ret.setTexture(tbd, body_diff)
        
        pivot = render.attachNewNode("ball")
        pivot.setPos(pos[0], pos[1], pos[2]) # Set location of pivot point
        ret.wrtReparentTo(pivot) # Preserve absolute position

        return pivot;


    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        KinematicModel.load_model(self, render, loader, color, scale)
        # self.agent_model = self.add_BB8(list(self.get_P()[:,0]), color, scale, render.attachNewNode('agent'))
        self.agent_model = self.add_sphere([self.goal[0], self.goal[1],0], color, scale, render.attachNewNode('agent'))
        self.goal_model = self.add_sphere([self.goal[0], self.goal[1],0], color[:-1]+[0.5], scale, render.attachNewNode('goal'))

    
    def redraw_model(self):
        self.agent_model.setPos(self.get_P()[0], self.get_P()[1], 0)
        self.goal_model.setPos(self.goal[0], self.goal[1], 0)
            
    
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
                    ball = loader.loadModel("resource/planet_sphere")
                    ball.setTransparency(TransparencyAttrib.MAlpha);
                    ball.setColor(0.1, 0.9, 0.1, 0.3);
                    ball.setScale(0.1);
                    ball.setPos(i,j,0);
                    ball.reparentTo(self.half_plane_handle[0])
