from .Ball3D import Ball3D
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy import cos, sin, arccos, sqrt, pi, arctan2

class HumanBall3D(Ball3D):

    """
    This the passive human 3D ball model. We assume a ball is control by human.
    The human has no response to the robot.
    """

    def __init__(self, agent, dT, auto = True, init_state=[5,5,0,0,0,0]):
        self.max_a = 4
        Ball3D.__init__(self, agent, dT, auto, init_state)
        self.RLS_cache['pred_delay'] = 3
        self.RLS_cache['A'] = self.A()
        self.RLS_cache['B'] = matrix(zeros((6, self.RLS_cache['pred_delay'] * 6 + 6)))
        self.RLS_cache['F'] = matrix(zeros((20, 20)))
        self.RLS_cache['alpha'] = 0.8
        self.RLS_cache['lambd'] = 0.98

    def rls_estimate_state(self, obstacle):
        u_cl = np.vstack([obstacle.m_his[:,-1], obstacle.m_his[:,-2], obstacle.m_his[:,-3], self.goal])
        phi = np.vstack([self.x_est, u_cl])
        A = self.RLS_cache['A']
        B = self.RLS_cache['B']
        F = self.RLS_cache['F']
        alpha = self.RLS_cache['alpha']
        lambd = self.RLS_cache['lambd']

        #\hat x(k|k-1)
        x_pred = A * self.x_est + B * u_cl
        #\hat x(k|k)
        x_est = (1 - alpha) * x_pred + alpha * self.observe(self.x)
        F = 1/lambd * ( F - (F * phi * phi.T * F) / (lambd + phi.T * F * phi))
        AB = np.hstack([A, B]) + (x_est - x_pred) * phi.T * F
        self.RLS_cache['A'] = AB[:,0:6]
        self.RLS_cache['B'] = AB[:,6:]
        self.RLS_cache['F'] = F
        #\hat x(k|k)
        self.x_est = x_est
        #\hat x(k+1|k), this x_pred is not the same as the above one
        self.x_pred = self.RLS_cache['A'] * self.x_est + self.RLS_cache['B'] * u_cl
        return x_est

    
    def update(self, obstacle):
        self.update_goal()
        self.m = self.x
        if self.auto:
            self.kalman_estimate_state()
        else:
            self.rls_estimate_state(obstacle)
        self.update_trace()

    def human_model(self):

        p = self.goal[[0,1,2]] - self.get_P()
        v = self.goal[[3,4,5]] - self.get_V()
        dis = np.linalg.norm(p)
        if dis > 2:
            self.u = p / max(abs(p)) * self.max_a
        elif 1 < dis and dis <= 2:
            self.u = (p / max(abs(p)) * 0.7 + v / max(abs(v)) * 0.3 ) * self.max_a + np.random.randn() * 0.05
        else:
            self.u = (p / max(abs(p)) * 0.6 + v / max(abs(v)) * 0.4 ) * self.max_a + np.random.randn() * 0.05
        

    def move(self, *args):
        if len(args) < 3:
            self.human_model()
        elif len(args) == 6:
            self.set_X(np.vstack(args))
            self.x_his = np.concatenate([self.x_his[:,1:], self.x],axis=1)
            return
        else:
            p = self.get_P()
            v = self.get_V()
            
            d = np.vstack(args)[[0,1]] - p
            dis = np.linalg.norm(d)
            if dis < 2:
                self.u = (d / 2 - v / self.max_v / 4) * self.max_a
            else:
                self.u = d / max(abs(d)) * self.max_a
        
        super().move()

        
    def load_model(self, render, loader, color=[0.8, 0.3, 0.2, 1], scale=0.5):
        Ball3D.load_model(self, render, loader, color, scale)
        # self.agent_model = self.add_sphere(list(self.get_P()[:,0]), color, scale, render.attachNewNode('agent'));
        # self.goal_model = self.add_sphere([self.goal[0], self.goal[1], self.goal[2]], color[:-1]+[0.5], scale, render.attachNewNode('goal'));

    def redraw_model(self):
        self.agent_model.setPos(self.get_P()[0], self.get_P()[1], self.get_P()[2])
        self.goal_model.setPos(self.goal[0], self.goal[1], self.goal[2])

    
    def model_auxiliary(self):
        if not hasattr(self, 'human_v'):
            [self.human_v, self.human_u] = self.draw_movement(list(self.get_PV()[:,0]), list(self.u[:,0])+[0])
        else:

            self.move_seg(self.human_u, np.array(self.get_P()).reshape(-1,).tolist(), np.array(self.u).reshape(-1,).tolist()+[0])
            self.move_seg(self.human_v, np.array(self.get_P()).reshape(-1,).tolist(), np.array(self.get_V()).reshape(-1,).tolist())
