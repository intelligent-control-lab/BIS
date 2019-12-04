from .KinematicModel import KinematicModel
import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix, diag, asscalar


from numpy import inf, cos, sin, arccos, sqrt, pi, arctan2, cross, dot, multiply

class RobotArm(KinematicModel):
    """
    This is the 4 DoF robot arm model.
    """

    def __init__(self, agent, dT, auto = True, init_state = [-4,-4, 0, 0, np.pi/2, -np.pi/2, -np.pi/2, 4.5, 4.5, 2]):
        self.max_v = pi/4
        self.max_a = pi
        self.k_v = 1
        self.max_ev = 2 
        self.tao_1 = np.matrix([[1, 1, 1], [0, 1, 1], [0, 0, 1]])
        self.tao_2 = np.matrix([[0, 1, 1], [0, 1, 1], [0, 0, 1]])
        self.tao_3 = np.matrix([[0, 0, 1], [0, 0, 1], [0, 0, 1]])
        KinematicModel.__init__(self, init_state, agent, dT, auto)

    def init_x(self, init_state):
        # x = [theta1, theta2, dot_theta1, dot_theta2]
        # u = [theta1_a, theta2_a]
        self.base = np.array(init_state[[0,1,2]]);
        self.x = matrix(zeros((8,1)));
        self.x[[0,1,2,3],0] = init_state[[3,4,5,6]];
        self.l = np.vstack(init_state[[7,8,9]]);
        self.u = matrix(zeros((4,1)))
        self.m = matrix(zeros((6,1)))

    def set_saturation(self):
        self.max_u =   matrix([self.max_a, self.max_a, self.max_a, self.max_a]).T;
        self.min_u = - self.max_u;
        self.max_x = matrix([pi*3/4, pi*3/4,   0,   0,  self.max_v,  self.max_v,  self.max_v,  self.max_v]).T
        self.min_x = matrix([-pi/4,  0, -pi*3/4, -pi*3/4, -self.max_v, -self.max_v, -self.max_v, -self.max_v]).T

    
    def aux_mat(self):
        theta1 = self.x[1]
        theta2 = self.x[2]
        theta3 = self.x[3]
        self.cos_v = np.vstack([cos(theta1), cos(theta1+theta2), cos(theta1+theta2+theta3)])
        self.sin_v = np.vstack([sin(theta1), sin(theta1+theta2), sin(theta1+theta2+theta3)])
        self.dot_theta_v = self.x[[5,6,7],0].copy()

    def get_P(self):
        self.aux_mat()
        self.aux_mat
        z = self.l.T * self.sin_v + self.base[2]
        r = self.l.T * self.cos_v
        x = r * cos(self.x[0]) + self.base[0]
        y = r * sin(self.x[0]) + self.base[1]
        return np.vstack([x,y,z]);
    
    def get_V(self):
        alpha = self.x[0]
        dot_alpha = self.x[4]
        self.aux_mat()
        r = self.l.T * self.cos_v
        dot_z = self.dot_theta_v.T * self.tao_1 * (np.multiply(self.l,  self.cos_v))
        dot_r = self.dot_theta_v.T * self.tao_1 * (np.multiply(self.l, -self.sin_v))
        dot_x = -r * sin(alpha) * dot_alpha + cos(alpha) * dot_r
        dot_y =  r * cos(alpha) * dot_alpha + sin(alpha) * dot_r
        return np.vstack([dot_x, dot_y, dot_z]);

    def get_ev(self, x):
        alpha = x[0]
        dot_alpha = x[4]
        
        theta1 = x[1]
        theta2 = x[2]
        theta3 = x[3]
        cos_v = np.vstack([cos(theta1), cos(theta1+theta2), cos(theta1+theta2+theta3)])
        sin_v = np.vstack([sin(theta1), sin(theta1+theta2), sin(theta1+theta2+theta3)])
        dot_theta_v = x[[5,6,7],0]

        r = self.l.T * cos_v
        dot_z = dot_theta_v.T * self.tao_1 * (np.multiply(self.l,  cos_v))
        dot_r = dot_theta_v.T * self.tao_1 * (np.multiply(self.l, -sin_v))
        dot_x = -r * sin(alpha) * dot_alpha + cos(alpha) * dot_r
        dot_y =  r * cos(alpha) * dot_alpha + sin(alpha) * dot_r

        return np.linalg.norm(np.vstack([dot_x, dot_y, dot_z]));

        
    #return the postion of closest point to obstacle
    def get_closest_X(self, Mh):
        Ph = np.squeeze(np.asarray(Mh[[0,1,2],0]))
        self.aux_mat()

        alpha = self.x[0,0]
        dot_alpha = self.x[4,0]
        drs = np.array(multiply(self.l, self.cos_v).T).reshape(-1,)
        dzs = np.array(multiply(self.l, self.sin_v).T).reshape(-1,)
        # print('self.l')
        # print(self.l)
        # print('self.cos_v')
        # print(self.cos_v)
        # print('self.sin_v')
        # print(self.sin_v)
        # print('self.x')
        # print(self.x)
        # print('drs')
        # print(drs)
        # print('cos(alpha)')
        # print(cos(alpha))

        dxs = drs * cos(alpha)
        dys = drs * sin(alpha)
        
        # print('dxs')
        # print(dxs)
        # print('dys')
        # print(dys)
        # print('dzs')
        # print(dzs)
        j1 = np.array([dxs[0], dys[0], dzs[0]]) + self.base
        j2 = j1 + np.array([dxs[1], dys[1], dzs[1]])
        j3 = j2 + np.array([dxs[2], dys[2], dzs[2]])


        # print('Ph')
        # print(Ph)
        # print('j1')
        # print(j1)
        # print('j2')
        # print(j2)
        # print('j3')
        # print(j3)
        # print('self.base')
        # print(self.base)

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
                dis = np.linalg.norm(cross(p1-x, p2-x)) / l;
                lm = dot(x - p1, p2 - p1) / np.linalg.norm(p2 - p1)
                return [dis, lm]

        [dis1, lm1] = point_2_seg(Ph, self.base, j1)
        [dis2, lm2] = point_2_seg(Ph, j1, j2)
        [dis3, lm3] = point_2_seg(Ph, j2, j3)

        if dis1 <= dis2 and dis1 <= dis3:
            lm2 = 0
            lm3 = 0
        elif dis2 <= dis1 and dis2 <= dis3:
            lm1 = self.l[0]
            lm3 = 0
        else:
            lm1 = self.l[0]
            lm2 = self.l[1]
        if lm1 < 1e-1:
            lm1 = 1e-1;
        
        lm = np.vstack([lm1, lm2, lm3])
        
        alpha = self.x[0]
        dot_alpha = self.x[4]
        self.aux_mat()

        rm = lm.T * self.cos_v
        mz = lm.T * self.sin_v + self.base[2]
        mx = rm * cos(self.x[0]) + self.base[0]
        my = rm * sin(self.x[0]) + self.base[1]

        dot_rm = self.dot_theta_v.T * self.tao_1 * (np.multiply(lm, -self.sin_v))
        dot_mz = self.dot_theta_v.T * self.tao_1 * (np.multiply(lm,  self.cos_v))
        dot_mx = -rm * sin(alpha) * dot_alpha + cos(alpha) * dot_rm
        dot_my =  rm * cos(alpha) * dot_alpha + sin(alpha) * dot_rm
        
        self.m = np.vstack([mx, my, mz, dot_mx, dot_my, dot_mz])

        self.lm = lm
        self.rm = rm
        self.dot_rm = dot_rm

        # print('self.m')
        # print(self.m)

        return self.m
    def Jacobbian(self, lm):
        self.aux_mat()
        alpha = self.x[0,0]
        dot_alpha = self.x[4,0]
        dot_rm = self.dot_rm
        t = self.dot_theta_v.T
        T1 = self.tao_1
        T2 = self.tao_2
        T3 = self.tao_3
        s = self.sin_v
        c = self.cos_v
        rm = lm.T * c
        E2 = matrix(diag([0, 1, 1]))
        E3 = matrix(diag([0, 0, 1]))
        lms = multiply(lm, s)
        lmc = multiply(lm, c)

        def unify(l):
            return np.array([np.asmatrix(x).item() for x in l])
        p_dot_mz_p_X = unify([0, t*T1*(-lms), t*T2*(-lms), t*T3*(-lms), 0,  lm.T*c,  lm.T*E2*c,  lm.T*E3*c])
        p_dot_rm_p_X = unify([0, t*T1*(-lmc), t*T2*(-lmc), t*T3*(-lmc), 0, -lm.T*s, -lm.T*E2*s, -lm.T*E3*s])
        p_rm_p_X = unify([0, -lm.T*s, -lm.T*E2*s, -lm.T*E3*s, 0, 0, 0, 0])
        p_mz_p_X = unify([0,  lm.T*c,  lm.T*E2*c,  lm.T*E3*c, 0, 0, 0, 0])
        p_alpha_p_X = np.array([1, 0, 0, 0, 0, 0, 0, 0])
        p_dot_alpha_p_X = np.array([0, 0, 0, 0, 1, 0, 0, 0])
        
        p_mx_p_X = cos(alpha) * p_rm_p_X - rm * sin(alpha) * p_alpha_p_X
        p_my_p_X = sin(alpha) * p_rm_p_X + rm * cos(alpha) * p_alpha_p_X
        p_dot_mx_p_X = p_alpha_p_X      * asscalar(-dot_alpha * rm * cos(alpha) - dot_rm * sin(alpha)) + \
                       p_rm_p_X         * asscalar(-dot_alpha * sin(alpha)) + \
                       p_dot_alpha_p_X  * asscalar(-rm * sin(alpha)) + \
                       p_dot_rm_p_X     * asscalar(cos(alpha))

        p_dot_my_p_X = p_alpha_p_X      * asscalar(-dot_alpha * rm * sin(alpha) + dot_rm * cos(alpha)) + \
                       p_rm_p_X         * asscalar(dot_alpha * cos(alpha)) + \
                       p_dot_alpha_p_X  * asscalar(rm * cos(alpha)) + \
                       p_dot_rm_p_X     * asscalar(sin(alpha))
        
        p_M_p_X = np.vstack([p_mx_p_X, p_my_p_X, p_mz_p_X, p_dot_mx_p_X, p_dot_my_p_X, p_dot_mz_p_X])
        
        return p_M_p_X
        
    def p_M_p_X(self):
        return self.Jacobbian(self.lm)
    
    def p_P_p_X(self):
        return self.Jacobbian(self.l)
        # print('rm')
        # print(rm)
        # print('sin(alpha)')
        # print(sin(alpha))
        # print('rm * sin(alpha)')
        # print(rm * sin(alpha))
        # print('cos(alpha)')
        # print(cos(alpha))
        # print('p_rm_p_X')
        # print(p_rm_p_X)
        # print('sin(alpha)')
        # print(sin(alpha))
        # print('p_alpha_p_X')
        # print(p_alpha_p_X)
        # print('rm')
        # print(rm)
        # print('sin(alpha)')
        # print(sin(alpha))
        # print('rm * sin(alpha)')
        # print(rm * sin(alpha))

    #TODO: This should be modified
    def set_P(self, theta):
        self.x[0,0] = theta[0];
        self.x[1,0] = theta[1];

    def set_V(self, dot_theta):
        self.x[2,0] = dot_theta[0];
        self.x[3,0] = dot_theta[1];


    def A(self):
        A = eye(8,8);
        A[0,4] = self.dT;
        A[1,5] = self.dT;
        A[2,6] = self.dT;
        A[3,7] = self.dT;
        return A;

    def B(self):
        B = matrix(zeros((8,4)));
        B[0,0] = self.dT**2/2;
        B[1,1] = self.dT**2/2;
        B[2,2] = self.dT**2/2;
        B[3,3] = self.dT**2/2;
        B[4,0] = self.dT;
        B[5,1] = self.dT;
        B[6,2] = self.dT;
        B[7,3] = self.dT;
        return B;

    def filt_u(self, u):
        u = np.minimum(u,  self.max_u);
        u = np.maximum(u, -self.max_u);
        return u;

    def filt_x(self, x):
        # x[0,0] = x[0,0] % (2*pi)
        # if x[0,0] < 0:
        #     x[0,0] = x[0,0] + 2*pi

        # x[1,0] = x[1,0] % (2*pi)
        # if x[1,0] < 0:
        #     x[1,0] = x[1,0] + 2*pi

        def height(x):
            sin_v = np.vstack([sin(x[1]), sin(x[1]+x[2]), sin(x[1]+x[2]+x[3])])
            dz = multiply(self.l, sin_v)
            z = [dz[0], dz[0]+dz[1], dz[0]+dz[1]+dz[2]] + self.base[2]
            return z
        
        while np.min(height(x)) < 0:
            x[1] = x[1] + 1e-3
        
        x = np.minimum(x,  self.max_x);
        x = np.maximum(x,  self.min_x);

        ev = np.linalg.norm(self.get_ev(x))
        while ev > self.max_ev:
            x[[4,5,6,7]] = x[[4,5,6,7]] * 0.9
            ev = np.linalg.norm(self.get_ev(x))

        return x

    
    def f(self, x, g):
        cos_v = np.vstack([cos(x[1]), cos(x[1]+x[2]), cos(x[1]+x[2]+x[3])])
        sin_v = np.vstack([sin(x[1]), sin(x[1]+x[2]), sin(x[1]+x[2]+x[3])])
        pz = self.l.T * sin_v + self.base[2]
        rp = self.l.T * cos_v
        px = rp * cos(x[0]) + self.base[0]
        py = rp * sin(x[0]) + self.base[1]
        d = np.linalg.norm(np.vstack([px,py,pz]) - g[[0,1,2]])
        return d
    
    def gradient_f(self, x, g):
        f = self.f(x, g)
        dx = 0.01
        df = np.zeros((4,1))
        for i in range(4):
            x2 = x
            x2[i,0] = x2[i,0] + dx
            df[i,0] = (self.f(x2, g) - f) / dx
        return df
    def inv_J(self):
        J = self.p_P_p_X()
        # inv_J = np.linalg.inv(J.T * J) * J.T
        # dot_p  = J * dot_x
        # J = p_dot_p_p_dot_x  -> p_P_p_X()[[3,4,5], [4,5,6,7]]
        # J * dot_x + dot_J * x = u = dot_dot_p
        dot_J = J[[3,4,5],:]
        J = J[[0,1,2],:]
        J = J[:,[0,1,2,3]]
        inv_J = J.T
        return inv_J

    def u_ref(self):
        inv_J = self.inv_J()
        dp = self.observe(self.goal[[0,1,2]] - self.get_P())
        dis = np.linalg.norm(dp)
        v = self.observe(self.get_V())
        df = self.gradient_f(self.observe(self.x[[0,1,2,3],0]), self.goal)
        
        self.k_v = 1
        if dis > 2:
            u0 = 0.2 * inv_J * dp - self.k_v * self.observe(self.x[[4,5,6,7],0]);
            # u0 = -df * sqrt(dis) / 10 - self.k_v * self.observe(self.x[[4,5,6,7],0]);
        else:
            u0 = 0.1 * inv_J * dp - 2 * self.k_v * self.observe(self.x[[4,5,6,7],0]);
            # u0 = -df * sqrt(dis) / 20 - self.k_v * self.observe(self.x[[4,5,6,7],0]);
        if self.get_P()[2] < 1e-3:
            u0[1] = self.max_a
        # print('u0')
        # print(u0)
        return u0;

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        KinematicModel.load_model(self, render, loader, color, scale)

        pos = self.base
        
        alpha = self.x[0,0] / np.pi * 180
        theta1 = self.x[1,0] / np.pi * 180
        theta2 = self.x[2,0] / np.pi * 180
        theta3 = self.x[3,0] / np.pi * 180
        l1 = self.l[0,0]
        l2 = self.l[1,0]
        l3 = self.l[2,0]

        self.agent_model = render.attachNewNode('agent')

        ret1 = loader.loadModel("resource/cube")
        ret1.reparentTo(self.agent_model)
        ret1.setColor(color[0], color[1], color[2], color[3]);
        ret1.setScale(l1/2, 0.1, 0.1);
        ret1.setPos(pos[0]+l1/2, pos[1], pos[2]);


        pivot1 = self.agent_model.attachNewNode("arm1-pivot")
        pivot1.setPos(pos[0], pos[1], pos[2]) # Set location of pivot point
        ret1.wrtReparentTo(pivot1) # Preserve absolute position
        

        ret2 = loader.loadModel("resource/cube")
        ret2.reparentTo(self.agent_model)
        ret2.setColor(color[0], color[1], color[2], color[3])
        ret2.setScale(l2/2, 0.1, 0.1)
        ret2.setPos(pos[0]+l1+l2/2, pos[1], pos[2])

        pivot2 = pivot1.attachNewNode("arm2-pivot")
        pivot2.setPos(l1, 0, 0) # Set location of pivot point
        ret2.wrtReparentTo(pivot2) # Preserve absolute position


        ret3 = loader.loadModel("resource/cube")
        ret3.reparentTo(self.agent_model)
        ret3.setColor(color[0], color[1], color[2], color[3])
        ret3.setScale(l3/2, 0.1, 0.1)
        ret3.setPos(pos[0]+l1+l2+l3/2, pos[1], pos[2])

        pivot3 = pivot2.attachNewNode("arm3-pivot")
        pivot3.setPos(l2, 0, 0) # Set location of pivot point
        ret3.wrtReparentTo(pivot3) # Preserve absolute position

        pivot1.setH(alpha)
        pivot1.setR(-theta1) # Rotates environ around pivot
        pivot2.setR(-theta2) # Rotates environ around pivot
        pivot3.setR(-theta3) # Rotates environ around pivot

        self.robot_arm1 = pivot1
        self.robot_arm2 = pivot2
        self.robot_arm3 = pivot3
        
        self.goal_model = self.add_sphere([self.goal[0], self.goal[1],0], color[:-1]+[0.5], scale);
        


    def redraw_model(self):
        self.goal_model.setPos(self.goal[0], self.goal[1], self.goal[2])
        self.robot_arm1.setH( self.x[0,0] / np.pi * 180);
        self.robot_arm1.setR(-self.x[1,0] / np.pi * 180);
        self.robot_arm2.setR(-self.x[2,0] / np.pi * 180);
        self.robot_arm3.setR(-self.x[3,0] / np.pi * 180);