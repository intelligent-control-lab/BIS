from .Ball import Ball
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy.random import rand, randn
from numpy.linalg import norm, inv
from numpy import cos, sin, arccos, sqrt, pi, arctan2

class HumanBall2D(Ball):

    """
    This the passive human 2D ball model. We assume a ball is control by human.
    The human has no response to the robot.
    """

    def __init__(self, agent, dT, auto = True, init_state=[5,5,0,0]):
        self.max_a = 4
        Ball.__init__(self, agent, dT, auto, init_state)
        self.RLS_cache['pred_delay'] = 3
        self.RLS_cache['A'] = self.A()
        self.RLS_cache['B'] = matrix(zeros((4, self.RLS_cache['pred_delay'] * 4 + 4)))
        self.RLS_cache['F'] = matrix(zeros((20, 20)))
        self.RLS_cache['alpha'] = 0.8
        self.RLS_cache['lambd'] = 0.98

    def rls_estimate_state(self, obstacle):
        idx_2d = [0,1,3,4]
        u_cl = np.vstack([obstacle.m_his[idx_2d,-1], obstacle.m_his[idx_2d,-2], obstacle.m_his[idx_2d,-3], self.goal[idx_2d]])
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
        self.RLS_cache['A'] = AB[:,0:4]
        self.RLS_cache['B'] = AB[:,4:]
        self.RLS_cache['F'] = F
        #\hat x(k|k)
        self.x_est = x_est
        #\hat x(k+1|k), this x_pred is not the same as the above one
        self.x_pred = self.RLS_cache['A'] * self.x_est + self.RLS_cache['B'] * u_cl
        return x_est

    
    def update(self, obstacle):
        self.update_goal()
        self.m[[0,1,3,4]] = self.x
        if self.auto:
            self.kalman_estimate_state()
        else:
            self.rls_estimate_state(obstacle)

    def human_model(self):

        p = (self.goal[[0,1,2]] - self.get_P())[[0,1]]
        v = (self.goal[[3,4,5]] - self.get_V())[[0,1]]
        dis = norm(p)
        if dis > 2:
            self.u = p / max(abs(p)) * self.max_a
        elif 1 < dis and dis <= 2:
            self.u = (p / max(abs(p)) * 0.7 + v / max(abs(v)) * 0.3 ) * self.max_a + randn() * 0.05
        else:
            self.u = (p / max(abs(p)) * 0.6 + v / max(abs(v)) * 0.4 ) * self.max_a + randn() * 0.05
        
    def move(self, *args):
        if len(args) < 3:
            self.human_model()
        elif len(args) == 6:
            x = np.vstack(args)
            x[2] = 0
            x[5] = 0
            self.set_X(x)
            self.x_his = np.concatenate([self.x_his[:,1:], self.x],axis=1)
            return
        else:
            p = self.get_P()[[0,1]]
            v = self.get_V()[[0,1]]
            
            d = np.vstack(args)[[0,1]] - p
            dis = norm(d)
            if dis < 2:
                self.u = (d / 2 - v / self.max_v / 4) * self.max_a
            else:
                self.u = d / max(abs(d)) * self.max_a
        
        super().move()

        
    def load_model(self, render, loader, color=[0.8, 0.3, 0.2, 1], scale=0.5):
        self.render = render
        self.agent_model = self.add_sphere(list(self.get_P()[:,0]), [0.8, 0.3, 0.2, 1], scale, render.attachNewNode('agent'));
        self.goal_model = self.add_sphere([self.goal[0], self.goal[1],0], [0.8, 0.3, 0.2, 0.5], scale, render.attachNewNode('goal'));

    def redraw_model(self):
        self.agent_model.setPos(self.get_P()[0], self.get_P()[1], 0)
        self.goal_model.setPos(self.goal[0], self.goal[1], 0)
        
    
    def model_auxiliary(self):
        if not hasattr(self, 'human_v'):
            [self.human_v, self.human_u] = self.draw_movement(list(self.get_PV()[:,0]), list(self.u[:,0])+[0])
        else:

            self.move_seg(self.human_u, np.array(self.get_P()).reshape(-1,).tolist(), np.array(self.u).reshape(-1,).tolist()+[0])
            self.move_seg(self.human_v, np.array(self.get_P()).reshape(-1,).tolist(), np.array(self.get_V()).reshape(-1,).tolist())

