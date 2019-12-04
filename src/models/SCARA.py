from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix


from numpy import inf, cos, sin, arccos, sqrt, pi, arctan2, cross, dot

class SCARA(KinematicModel):
    """
    This is the SCARA model, a 2 DoF plane robot.
    """

    def __init__(self, agent, dT, auto = True, init_state = [-4,-4, 0, 0, 4.5, 4.5]):
        self.max_v = pi/4
        self.max_a = 2*pi
        self.k_v = 1
        self.max_ev = 2 
        KinematicModel.__init__(self, init_state, agent, dT, auto, is_2D = True);
    def init_x(self, init_state):
        # x = [theta1, theta2, dot_theta1, dot_theta2]
        # u = [theta1_a, theta2_a]
        self.base = init_state[[0,1]];
        self.x = matrix(zeros((4,1)));
        self.x[[0,1],0] = init_state[[2,3]];
        self.l = init_state[[4,5]];
        self.u = matrix(zeros((2,1)))

    def set_saturation(self):
        self.max_u =   matrix([self.max_a, self.max_a]).T;
        self.min_u = - self.max_u;
        self.max_x =   matrix([inf, inf, self.max_v, self.max_v]).T
        self.min_x = - self.max_x

    #return the postion of end point
    def get_P(self):
        l1 = self.l[0]
        l2 = self.l[1]
        theta1 = self.x[0]
        theta2 = self.x[1]
        px = l1 * cos(theta1) + l2 * cos(theta1 + theta2) + self.base[0]
        py = l1 * sin(theta1) + l2 * sin(theta1 + theta2) + self.base[1]
        return np.vstack([px, py, 0]);

    def get_V(self):
        l1 = self.l[0]
        l2 = self.l[1]
        theta1 = self.x[0]
        theta2 = self.x[1]
        vx = (-l1 * sin(theta1) - l2 * sin(theta1 + theta2)) * self.x[2]  - l2 * sin(theta1 + theta2) * self.x[3];
        vy = l1 * cos(theta1) * self.x[2] + l2 * cos(theta1 + theta2) * self.x[3];
        return np.vstack([vx, vy, 0]);

    #return the postion of closest point to obstacle
    def get_closest_X(self, Xh):
        Ph = np.squeeze(np.asarray(Xh[[0,1],0]))

        l1 = self.l[0]
        l2 = self.l[1]
        theta1 = self.x[0,0]
        theta2 = self.x[1,0]
        
        j1 = np.array([l1 * cos(theta1), l1 * sin(theta1)])
        j2 = np.array([l1 * cos(theta1) + l2 * cos(theta1 + theta2), 
                       l1 * sin(theta1) + l2 * sin(theta1 + theta2)])
        
        j1 = j1 + self.base
        j2 = j2 + self.base

        def is_between(x, p1, p2):
            return dot(x - p1, p2 - p1) > 0 and dot(x - p2, p1 - p2) > 0

        def point_2_seg(x, p1, p2):
            d1 = np.linalg.norm(x - p1)
            d2 = np.linalg.norm(x - p2)
            l = np.linalg.norm(p1 - p2)
            if not is_between(x, p1, p2):
                if d1 < d2:
                    return [d1, 0]
                else:
                    return [d2, l]
            else:
            
                dis = abs(cross(p1-x, p2-x)) / l;
                lm = dot(x - p1, p2 - p1) / np.linalg.norm(p2 - p1)
                return [dis, lm]

        [dis1, lm1] = point_2_seg(Ph, self.base, j1)
        [dis2, lm2] = point_2_seg(Ph, j1, j2)
        if dis1 < dis2:
            lm2 = 0
        else:
            lm1 = self.l[0]
        if lm1 < 1e-1:
            lm1 = 1e-1;
        mx = lm1 * cos(theta1) + lm2 * cos(theta1 + theta2) + self.base[0]
        my = lm1 * sin(theta1) + lm2 * sin(theta1 + theta2) + self.base[1]

        dot_mx = (-lm1 * sin(theta1) - lm2 * sin(theta1 + theta2)) * self.x[2]  - lm2 * sin(theta1 + theta2) * self.x[3];
        dot_my = lm1 * cos(theta1) * self.x[2] + lm2 * cos(theta1 + theta2) * self.x[3];
        
        self.m = np.vstack([mx, my, 0, dot_mx, dot_my, 0])

        self.lm = [lm1, lm2]
        return self.m

    def Jacobbian(self, lm):
        theta1 = self.x[0]
        theta2 = self.x[1]
        lm1 = lm[0]
        lm2 = lm[1]
        ret = matrix(zeros((6,4)));
        ret[0,0] = -lm1 * sin(theta1) - lm2 * sin(theta1 + theta2)
        ret[0,1] = -lm2 * sin(theta1 + theta2)
        ret[1,0] = lm1 * cos(theta1) + lm2 * cos(theta1 + theta2)
        ret[1,1] = lm2 * cos(theta1 + theta2)
        ret[3,0] = (-lm1 * cos(theta1) - lm2 * cos(theta1 + theta2)) * self.x[2]  - lm2 * cos(theta1 + theta2) * self.x[3]
        ret[3,1] = (-lm2 * cos(theta1 + theta2)) * self.x[2]  - lm2 * cos(theta1 + theta2) * self.x[3]
        ret[3,2] = -lm1 * sin(theta1) - lm2 * sin(theta1 + theta2)
        ret[3,3] = -lm2 * sin(theta1 + theta2)
        ret[4,0] = -lm1 * sin(theta1) * self.x[2] - lm2 * sin(theta1 + theta2) * self.x[3]
        ret[4,1] = -lm2 * sin(theta1 + theta2) * self.x[3]
        ret[4,2] = lm1 * cos(theta1)
        ret[4,3] = lm2 * cos(theta1 + theta2)
        return ret

    def p_M_p_X(self):
        return self.Jacobbian(self.lm)
    
    def p_P_p_X(self):
        return self.Jacobbian(self.l)

    #TODO: This should be modified
    def set_P(self, theta):
        self.x[0,0] = theta[0];
        self.x[1,0] = theta[1];

    def set_V(self, dot_theta):
        self.x[2,0] = dot_theta[0];
        self.x[3,0] = dot_theta[1];


    def A(self):
        A = eye(4,4);
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

    def filt_u(self, u):
        u = np.minimum(u,  self.max_u);
        u = np.maximum(u, -self.max_u);
        return u;


    def get_ev(self, x):
        l1 = self.l[0]
        l2 = self.l[1]
        theta1 = x[0]
        theta2 = x[1]
        vx = (-l1 * sin(theta1) - l2 * sin(theta1 + theta2)) * x[2]  - l2 * sin(theta1 + theta2) * x[3];
        vy = l1 * cos(theta1) * x[2] + l2 * cos(theta1 + theta2) * x[3];
        return np.linalg.norm(np.vstack([vx, vy]));

    def filt_x(self, x):
        # x[0,0] = x[0,0] % (2*pi)
        # if x[0,0] < 0:
        #     x[0,0] = x[0,0] + 2*pi

        # x[1,0] = x[1,0] % (2*pi)
        # if x[1,0] < 0:
        #     x[1,0] = x[1,0] + 2*pi
        x = np.minimum(x,  self.max_x);
        x = np.maximum(x,  self.min_x);

        ev = np.linalg.norm(self.get_ev(x))
        while ev > self.max_ev:
            x[[2,3]] = x[[2,3]] * 0.9
            ev = np.linalg.norm(self.get_ev(x))

        return x

    def inv_J(self):
        J = self.Jacobbian(self.l)
        J = J[[0,1],:]
        J = J[:,[0,1]]
        return J.T
        # a = self.l[0]
        # b = self.l[1]
        # c = np.linalg.norm(p[[0,1]])
        # a_theta2 = arccos((a**2 + b**2 - c**2) / (2*a*b))
        # a_theta2 = a_theta2 - pi
        # d1 = arccos((a**2 + c**2 - b**2) / (2*a*c))
        # d2 = arctan2(p[1], p[0])
        # a_theta1 = d1 + d2;

        # c_theta1 = d2 - d1;
        # c_theta2 = 2*pi - a_theta2;
        
        # if abs(a_theta1 - self.x[0]) > pi:
        #     a_theta1 = (a_theta1 - 2*pi)
        # if abs(a_theta2 - self.x[1]) > pi:
        #     a_theta2 = (a_theta2 - 2*pi)

        # if abs(c_theta1 - self.x[0]) > pi:
        #     c_theta1 = (c_theta1 - 2*pi)
        # if abs(c_theta2 - self.x[1]) > pi:
        #     c_theta2 = (c_theta2 - 2*pi)
        
        # dis1 = (self.x[0] - a_theta1)**2
        # dis2 = (self.x[0] - c_theta1)**2

        
        # if dis1 < dis2:
        #     return [a_theta1, a_theta2]
        # else:
        #     return [c_theta1, c_theta2]

    def u_ref(self):
        inv_J = self.inv_J()
        dp = self.observe((self.goal - self.get_PV())[[0,1]]);
        dis = np.linalg.norm(dp);
        v = self.observe(self.get_V())[[0,1]];

        if dis > 2:
            u0 = 0.2 * inv_J * dp - self.k_v * self.observe(self.x[[2,3],0]);
        else:
            u0 = 0.1 * inv_J * dp - self.k_v * self.observe(self.x[[2,3],0]);
        


        return u0;

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        KinematicModel.load_model(self, render, loader, color, scale)

        pos = [-4, -4, 0]
        theta1 = 0;
        theta2 = 0;
        l1 = 4.5;
        l2 = 4.5;

        self.agent_model = render.attachNewNode('agent')

        ret1 = loader.loadModel("resource/cube")
        ret1.reparentTo(self.agent_model)
        ret1.setColor(color[0], color[1], color[2], color[3]);
        ret1.setScale(l1/2, 0.1, 0.1);
        ret1.setPos(pos[0]+l1/2, pos[1], pos[2]);

        pivot1 = self.agent_model.attachNewNode("arm1-pivot")
        pivot1.setPos(pos[0], pos[1], pos[2]) # Set location of pivot point
        ret1.wrtReparentTo(pivot1) # Preserve absolute position
        pivot1.setH(theta1) # Rotates environ around pivot

        ret2 = loader.loadModel("resource/cube")
        ret2.reparentTo(self.agent_model)
        ret2.setColor(color[0], color[1], color[2], color[3]);
        ret2.setScale(l2/2, 0.1, 0.1);
        ret2.setPos(pos[0]+l2/2+l1, pos[1], pos[2]);

        pivot2 = pivot1.attachNewNode("arm2-pivot")
        pivot2.setPos(l1, 0, 0) # Set location of pivot point
        ret2.wrtReparentTo(pivot2) # Preserve absolute position
        pivot2.setH(theta2) # Rotates environ around pivot

        self.robot_arm1 = pivot1
        self.robot_arm2 = pivot2

        self.goal_model = self.add_sphere([self.goal[0], self.goal[1],0], color[:-1]+[0.5], scale);
        
        
    def redraw_model(self):
        self.robot_arm1.setH(self.x[0,0] / np.pi * 180);
        self.robot_arm2.setH(self.x[1,0] / np.pi * 180);

        self.goal_model.setPos(self.goal[0], self.goal[1], 0)
