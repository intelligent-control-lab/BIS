import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix
from numpy import cos, sin, arccos, sqrt, pi, arctan2
from panda3d.core import *
from direct.gui.DirectGui import *
from utils.ArgsPack import ArgsPack

class KinematicModel:
    """This is the base class for all robot dynamic models. 
    We assume the models are all in the form:

    :math:`X' = A * X +  B * u`

    :math:`\dot X  =  fx + fu * u`
    
    Because

    :math:`X' = X + \dot X * dT`

    Then

    :math:`fx = (A - I) / dT`

    :math:`fu = B / dT`

    We just need to specify A and B to define different dynamic models.

    There are two major phases in the control circle, update and move. In the update phase, the robot will update its information based on the environment. And in the move phase, the robot will execute control input.
    """

    def __init__(self, init_state, agent, dT, auto, is_2D=False):
        """This function initilize the robot.
        
        Args:
            init_state (list): the init state of the robot, for example [x, y, vx, vy]
            agent (MobileAgent()): the algorithm that controls this robot.
            dT (float): the seperation between two control output
            auto (bool): whether this robot is autonomous, if not, it is control by user input like mouse.
            is_2D (bool): whether this model is a 2D model, which means it can only move on the groud plane.

        """
        self.control_noise = 0.02 # noise scale
        self.safe_dis = 1
        self.map_size = 10 # map boundary size
        self.fraction = 0.2 # velocity decrease rate per dT
        self.disk_radius = 0.4 # radius of disk
        self.measure_noise = 0.02 # noise scale
        self.auto=False # whether is controled by human
        self.RLS_cache = dict() # RLS cache

        self.init_state = np.array(init_state)
        self.set_saturation()

        self.dT = dT
        
        self.agent = agent
        self.auto = auto
        self.is_2D = is_2D
        goals = np.stack([np.random.rand(100)* self.map_size/2 - self.map_size / 4, 
                        np.random.rand(100)* self.map_size/2 - self.map_size / 4, 
                        np.random.rand(100)* self.map_size/4 + self.map_size / 4,
                        zeros(100), 
                        zeros(100),
                        zeros(100)], axis=0 )

        self.reset(dT, goals)

        self.get_closest_X(np.vstack([10,10,10,0,0,0]))

    def reset(self, dT, goals):
        """This function reset the robot state to initial, and set the goals to given goals. This function is useful when the user need to make sure all the robot are tested under the same goal sequence,
        
        Args:
            dT (float): the seperation between two control output
            goals (ndarray): n*6 array of goal specification. [x y z 0 0 0]
        """

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

        self.get_closest_X(np.vstack([10,10,10,0,0,0]))

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
        if np.linalg.norm(dx) < 0.3 and np.linalg.norm(dv) < 0.5:
            self.goal_achieved = self.goal_achieved + 1
            self.goal = np.vstack(self.goals[:,self.goal_achieved])

    def observe(self, x):
        return x + np.random.randn(*np.shape(x)) * self.measure_noise

    def get_PV(self):
        """This function return the cartesian position and velocity of the robot,
        
        Returns:
            PV (ndarray): 6*1 array. [x y z vx vy vz]
        """
        return np.vstack([self.get_P(), self.get_V()])

    def set_X(self, x):
        self.set_P(x[[0,1,2]])
        self.set_V(x[[3,4,5]])


    
    def fx(self):
        """
        This function calculate fx from A,
        Because
        X' = X + dot_X * dT
        Then
        fx = (A - I) / dT
        """
        return (self.A() - np.eye(np.shape(self.x)[0])) / self.dT * self.x
    def fu(self):
        """
        This function calculate fu from B,
        Because
        X' = X + dot_X * dT
        Then
        fu = B / dT
        """
        return self.B() / self.dT

    def filt_u(self, u):
        """return the saturated control input based the given reference control input

        Args:
            u (ndarray): reference control input

        Returns:
            u (ndarray): saturated control input
        """
        u = np.minimum(u,  self.max_u)
        u = np.maximum(u,  self.min_u)
        return u
        
    def filt_x(self, x):
        """return the saturated robot state based the given reference state

        Args:
            x (ndarray): reference state

        Returns:
            x (ndarray): saturated state
        """
        x = np.minimum(x,  self.max_x)
        x = np.maximum(x,  self.min_x)
        return x

    def update_score(self, obstacle):
        """Update the scores of the robot based on the relative postion and relative velocity to the obstacle. The scores are used to draw roc curves and generate statistical restuls.

        Args:
            obstacle (KinematicModel()): the obstacle
        """
        dm = obstacle.m - self.m
        dp = (obstacle.m - self.m)[[0,1,2],0]
        dv = (obstacle.m - self.m)[[3,4,5],0]
        dis = np.linalg.norm(dp)
        v_op = np.asscalar(dv.T * dp / dis)

        if dis < self.safe_dis:
            if self.time - self.last_collision_time > 5:

                self.score['collision_cnt'] = self.score['collision_cnt'] + 1
            self.last_collision_time = self.time
        
        if v_op < 0 and dis < 2*self.safe_dis:
            self.score['safety'] = self.score['safety'] + min(0, np.log(dis / (2 * self.safe_dis) + 1e-20)) * abs(v_op);

            # self.score['safety'] = self.score['safety'] + min(2 * self.safe_dis, dis);
        
        self.score['nearest_dis'] = min(self.score['nearest_dis'], dis)

        self.score['efficiency'] = self.goal_achieved
        

    def update(self, obstacle):
        """Update phase. 1. update score, 2. update goal, 3. update self state estimation, 4. update the nearest point on self to obstacle, 5. calculate control input, 6. update historical trajectory.

        Args:
            obstacle (KinematicModel()): the obstacle
        """
        self.time = self.time + 1
        self.update_score(obstacle)
        self.update_goal()
        self.kalman_estimate_state()
        self.update_m(obstacle.m)
        self.calc_control(obstacle)
        self.update_trace()

    def update_trace(self):
        """
        update trace of end effector
        """
        self.trace = np.concatenate([self.trace[:,1:], self.get_P()],axis=1)
        
    def update_m(self, Mh):
        """Update the nearest cartesian point on self to obstacle. 
        Args:
            Mh (ndarray): 6*1 array, cartesian postion and velocity of the obstacle.
        """
        self.m = self.get_closest_X(Mh)

    def kalman_estimate_state(self):
        """
        Use kalman filter to update the self state estimation.
        """
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
        K = P * H.T * np.linalg.inv(S)
        x_est = x_pred + K * y
        P = (I - K*H) * P * (I - K*H).T + K * R * K.T
        self.kalman_P = P
        self.x_est = self.filt_x(x_est) # \hat x(k|k)
        self.x_pred = self.filt_x(A * self.x_est + B * self.u) # \hat x(k+1|k)
        return x_est

    def calc_control(self, obstacle):
        """
        Generate control input by the agent.

        Args:
            obstacle (KinematicModel()): the obstacle
        """
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
        self.u = self.agent.calc_control_input(dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u)
        self.u = self.filt_u(self.u)

    def dot_X(self):
        """
        First order estimation of dot_X using current state and last state.
        """
        return (self.x - self.x_his[:,-2]) / self.dT
        # return (self.x_pred - self.x_est) / self.dT
        
    def move(self):
        """
        Move phase. An random disturbance is added to the control input.
        """
        self.x = self.A() * self.x + self.B() * (self.u + np.random.randn(np.shape(self.u)[0],1) * self.control_noise)
        self.x = self.filt_x(self.x)
        self.x_his = np.concatenate([self.x_his[:,1:], self.x],axis=1)
        self.m_his = np.concatenate([self.m_his[:,1:], self.m], axis=1)

# The following functions are required to fill up for new models.

    def init_x(self, init_state):
        """
        init state x
        """
        pass
    def set_saturation(self):
        """
        Set min and max cut off for state x and control u.
        """
        pass
    def get_P(self):
        """
        Return position in the Cartisian space.
        """
        pass
    def get_V(self):
        """
        Return velocity in the Cartisian space.
        """
        pass
    def set_P(self, p):
        """
        Set position in the Cartisian space.

        Args:
            p (ndarray): position
        """
        pass
    def set_V(self, v):
        """
        Set velocity in the Cartisian space

        Args:
            v (ndarray): velocity
        """
        pass
    
    def A(self):
        """
        Transition matrix A as explained in the class definition.
        """
        pass
    def B(self):
        """
        Transition matrix B as explained in the class definition.
        """
        pass
    def get_closest_X(self, Mh):
        """
        Update the corresponding state of the nearest cartesian point on self to obstacle. 
        
        Args:
            Mh (ndarray): 6*1 array, cartesian postion and velocity of the obstacle.
        """
        pass
    def p_M_p_X(self): # p closest point p X
        """ dM / dX, the derivative of the nearest cartesian point to robot state.
        """
        pass
    def estimate_state(self):
        """
        State estimater caller.
        """
        pass
    def u_ref(self):
        """
        Reference control input.
        """
        pass


############## Graphics ##############

    def add_sphere(self, pos, color, scale=0.5, render_node=None):
        """
        Add a sphere model into the scene.

        Args:
            pos: position to place the sphere
            color: color of the sphere
            scale: scale to zoom the sphere
        """
        if render_node is None:
            render_node = self.render
        ret = loader.loadModel("resource/planet_sphere")
        ret.reparentTo(render_node)
        ret.setTransparency(TransparencyAttrib.MAlpha)
        ret.setColor(color[0], color[1], color[2], color[3])
        ret.setScale(scale)
        ret.setPos(pos[0], pos[1], pos[2])
        return ret;

    def draw_trace(self):
        """
        Show the trace of the end effector.
        """

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

    # def draw_arrow(self, p_from, p_to, color):
    #     segs = LineSegs( )
    #     segs.setThickness( 20.0 )
    #     segs.setColor( color )
    #     segs.moveTo( p_from )
    #     segs.drawTo( p_to )
    #     arrow = segs.create( )
    #     self.render.attachNewNode(arrow)
    #     return segs

    def draw_movement(self, X, u):
        """
        For debug use.
        Show the velocity vector and control vector.
        """
        p_from = LVector3f(X[0], X[1], X[2]);
        v_to = p_from + LVector3f(X[3], X[4], X[5]);
        u_to = p_from + LVector3f(u[0], u[1], u[2]);
        u_color = Vec4(0.2, 0.8, 0.2, 0.5);
        v_color = Vec4(0.8, 0.2, 0.8, 0.5);
        return [self.draw_arrow(p_from, v_to, v_color), self.draw_arrow(p_from, u_to, u_color)];

    def move_seg(self, vdata, p_from, vec):
        """
        Move a segment line to a new position.

        Args:
            vdata: segment line handle
            p_from: new start point
            vec: the segment line vector
        """
        p_from = LVector3f(p_from[0], p_from[1], p_from[2])
        p_to = p_from + LVector3f(vec[0], vec[1], vec[2])
        vdata.setVertex(0, p_from)
        vdata.setVertex(1, p_to)  

# The following functions are required to fill up for new models.

    def load_model(self, render, loader, color=[0.1, 0.5, 0.8, 1], scale=0.5):
        """
        Load the 3d model to be shown in the GUI

        Args:
            render : panda3d component
            loader : panda3d component
            color (list): RGB and alpha
            scale (float): scale to zoom the loaded 3d model.
        """
        self.color = color
        self.render = render

    def redraw_model(self):
        """
        Refresh the position of the robot model and goal model in the GUI.
        """
        pass
    def model_auxiliary(self):
        """
        This function is for debug use.
        """
        pass