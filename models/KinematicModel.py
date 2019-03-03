import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy.random import rand, randn
from numpy.linalg import norm, inv
from numpy import cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *
from utils.ArgsPack import ArgsPack

class KinematicModel:

    control_noise = 0.02 # noise scale

    safe_dis = 1
    
    map_size = 10 # map boundary size
    fraction = 0.2 # velocity decrease rate per dT

    disk_radius = 0.4 # radius of disk

    measure_noise = 0.02 # noise scale
    # auto # whether is controled by human
    # kalman_P
    RLS_cache = dict()

    def __init__(self, init_state, agent, dT, auto, is_2D=False):
        self.init_state = np.array(init_state)
        self.set_saturation()

        self.dT = dT
        
        self.agent = agent
        self.auto = auto
        self.is_2D = is_2D
        goals = np.stack([rand(100)* self.map_size/2 - self.map_size / 4, 
                        rand(100)* self.map_size/2 - self.map_size / 4, 
                        rand(100)* self.map_size/4 + self.map_size / 4,
                        zeros(100), 
                        zeros(100),
                        zeros(100)], axis=0 )

        self.reset(dT, goals)

    def reset(self, dT, goals):
        self.dT = dT
        self.set_goals(goals)

        self.init_x(self.init_state)
        self.x_his = repmat(self.x, 1, 50)
        self.n = np.shape(self.x)[0]
        self.H = matrix(eye(self.n))
        self.kalman_P = matrix(eye(self.n)) * (self.measure_noise**2)
        self.x_est = self.observe(self.x)
        self.m = matrix(zeros((6,1)))
        self.m_his = repmat(self.m, 1, 50)
        self.x_pred = zeros((self.n,1))
        self.trace = repmat(self.get_P(), 1, 100)

        self.goal_achieved = 0

        self.time = 0
        self.last_collision_time = 0
        self.score = dict()
        self.score['collision_cnt'] = 0
        self.score['safety'] = 0
        self.score['nearest_dis'] = 1e9
        self.score['efficiency'] = 0
        self.predictability = 0

    def set_goals(self, goals):
        self.goals = goals
        if self.is_2D:
            self.goals[2,:] = zeros(100)
        self.goal_achieved = 0
        self.goal = np.vstack(self.goals[:,0])

    def update_goal(self):
        dx = self.get_P() - self.goal[[0,1,2]]
        dv = self.get_V() - self.goal[[3,4,5]]
        # print('dx')
        # print(dx)
        # print('dv')
        # print(dv)
        if norm(dx) < 0.3 and norm(dv) < 0.5:
            self.goal_achieved = self.goal_achieved + 1
            self.goal = np.vstack(self.goals[:,self.goal_achieved])

    def observe(self, x):
        return x + randn(*np.shape(x)) * self.measure_noise

    def get_PV(self):
        return np.vstack([self.get_P(), self.get_V()])

    def set_X(self, x):
        self.set_P(x[[0,1,2]])
        self.set_V(x[[3,4,5]])


    # We assume all the models are in the form
    #     X =  A * X +  B * u
    # dot_X =     fx + fu * u
    def fx(self):
        return (self.A() - np.eye(np.shape(self.x)[0])) / self.dT * self.x
    def fu(self):
        return self.B() / self.dT

    def filt_u(self, u):
        u = np.minimum(u,  self.max_u)
        u = np.maximum(u,  self.min_u)
        return u
        
    def filt_x(self, x):
        x = np.minimum(x,  self.max_x)
        x = np.maximum(x,  self.min_x)
        return x

    def update_score(self, obstacle):
        dm = obstacle.m - self.m
        dp = (obstacle.m - self.m)[[0,1,2],0]
        dv = (obstacle.m - self.m)[[3,4,5],0]
        dis = norm(dp)
        v_op = np.asscalar(dv.T * dp / dis)

        if dis < self.safe_dis:
            if self.time - self.last_collision_time > 5:

                self.score['collision_cnt'] = self.score['collision_cnt'] + 1
            self.last_collision_time = self.time
        
        if v_op < 0:
            self.score['safety'] = self.score['safety'] + min(0, np.log(dis / (2 * self.safe_dis) + 1e-20)) * abs(v_op);

            # self.score['safety'] = self.score['safety'] + min(2 * self.safe_dis, dis);
        
        self.score['nearest_dis'] = min(self.score['nearest_dis'], dis)

        self.score['efficiency'] = self.goal_achieved
        

    def update(self, obstacle):
        self.time = self.time + 1
        self.update_score(obstacle)
        self.update_goal()
        self.kalman_estimate_state()
        self.update_m(obstacle.m)
        self.calc_control(obstacle)
        self.update_trace()

    def update_trace(self):
        self.trace = np.concatenate([self.trace[:,1:], self.get_P()],axis=1)
        
    def update_m(self, Mh):
        self.m = self.get_closest_X(Mh)

    def kalman_estimate_state(self):
        dT = self.dT
        A = self.A()
        B = self.B()
        Q = B * B.T * (self.control_noise)**2 # adopt max_a / 2 as sigma. because 95# percent of value lie in mu-2*sigma to mu+2*sigma
        R = matrix(eye(self.n)) * (self.measure_noise**2)
        I = matrix(eye(self.n))
        P = self.kalman_P
        H = self.H
        x_pred = A * self.x_est + B * self.u
        P = A * P * A.T + Q
        z = self.observe(self.x)
        y = z - self.H * x_pred
        S = R + H * P * H.T
        K = P * H.T * inv(S)
        x_est = x_pred + K * y
        P = (I - K*H) * P * (I - K*H).T + K * R * K.T
        self.kalman_P = P
        self.x_est = self.filt_x(x_est) # \hat x(k|k)
        self.x_pred = self.filt_x(A * self.x_est + B * self.u) # \hat x(k+1|k)
        return x_est

    def calc_control(self, obstacle):
        
        dT = self.dT
        goal = self.goal
        fx = self.fx()
        fu = self.fu()
        Xr = self.x_est
        Xh = obstacle.x_est
        Mr = self.m
        Mh = obstacle.m
        dot_Xr = self.dot_X()
        dot_Xh = obstacle.dot_X()            
        p_Mr_p_Xr = self.p_M_p_X()
        p_Mh_p_Xh = obstacle.p_M_p_X()
        u0 = self.u_ref()
        min_u = self.min_u
        max_u = self.max_u
        self.u = self.agent.calc_control_input(dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u);
        self.u = self.filt_u(self.u)

    def dot_X(self):
        return (self.x - self.x_his[:,-2]) / self.dT
        # return (self.x_pred - self.x_est) / self.dT
        
    def move(self):
        self.x = self.A() * self.x + self.B() * (self.u + randn(np.shape(self.u)[0],1) * self.control_noise)
        self.x = self.filt_x(self.x)
        self.x_his = np.concatenate([self.x_his[:,1:], self.x],axis=1)
        self.m_his = np.concatenate([self.m_his[:,1:], self.m], axis=1)

    def init_x(self, init_state):
        pass
    def set_saturation(self):
        pass
    def get_P(self):
        pass
    def get_V(self):
        pass
    def set_P(self, p):
        pass
    def set_V(self, v):
        pass
    # We assume all the models are in the form
    #     X =  A * X +  B * u
    # dot_X =     fx + fu * u
    def A(self):
        pass
    def B(self):
        pass
    def get_closest_X(self, Xh):
        pass
    def p_M_p_X(self): # p closest point p X
        pass
    def estimate_state(self):
        pass
    def u_ref(self):
        pass


############## Graphics ##############

    def add_sphere(self, pos, color, scale=0.5):
        ret = loader.loadModel("resource/planet_sphere")
        ret.reparentTo(self.render)
        ret.setTransparency(TransparencyAttrib.MAlpha)
        ret.setColor(color[0], color[1], color[2], color[3])
        ret.setScale(scale)
        ret.setPos(pos[0], pos[1], pos[2])
        return ret;

    def draw_trace(self):
        if hasattr(self, 'trace_line_handle'):
            self.trace_line_handle.removeNode()

        segs = LineSegs( )
        segs.setThickness( 5.0 )
        segs.setColor(self.color[0], self.color[1], self.color[2], 0)
        
        p_from = LVector3f(self.trace[0,0], self.trace[1,0], self.trace[2,0])
        segs.moveTo( p_from )
        for i in range(np.shape(self.trace)[1]):
            p_to = LVector3f(self.trace[0,i], self.trace[1,i], self.trace[2,i])
            segs.setColor(self.color[0], self.color[1], self.color[2], 0)
            segs.drawTo( p_to )
        trace_line = segs.create( )

        self.trace_line_handle = self.render.attachNewNode(trace_line);

    def draw_arrow(self, p_from, p_to, color):
        segs = LineSegs( )
        segs.setThickness( 20.0 )
        segs.setColor( color )
        segs.moveTo( p_from )
        segs.drawTo( p_to )
        arrow = segs.create( )
        self.render.attachNewNode(arrow)
        return segs

    def draw_movement(self, X, u):
        p_from = LVector3f(X[0], X[1], X[2]);
        v_to = p_from + LVector3f(X[3], X[4], X[5]);
        u_to = p_from + LVector3f(u[0], u[1], u[2]);
        u_color = Vec4(0.2, 0.8, 0.2, 0.5);
        v_color = Vec4(0.8, 0.2, 0.8, 0.5);
        return [self.draw_arrow(p_from, v_to, v_color), self.draw_arrow(p_from, u_to, u_color)];
    
    def move_seg(self, vdata, p_from, vec):
        p_from = LVector3f(p_from[0], p_from[1], p_from[2])
        p_to = p_from + LVector3f(vec[0], vec[1], vec[2])
        vdata.setVertex(0, p_from)
        vdata.setVertex(1, p_to)  

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 0.8], scale=0.5):
        self.color = color
        self.render = render

    def redraw_model(self):
        pass
    def model_auxiliary(self):
        pass