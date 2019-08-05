import numpy as np
from numpy.matlib import repmat
from numpy import zeros, eye, ones, matrix

from numpy import cos, sin, arccos, sqrt
from models import *

class MobileAgent:

    """This is the base class for algorithms. We assume all the algorithms use relative cartesian position and velocity to evaluate current safety index.
    """

    def __init__(self):
        pass

    def calc_control_input(self, dT, goal, fx, fu, Xr, Xh, dot_Xr, dot_Xh, Mr, Mh, p_Mr_p_Xr, p_Mh_p_Xh, u0, min_u, max_u):
        """This function return a control input based on internal state and obstacle information

        Args:
            dT (float): dT
            goal (ndarray): [x; y; z; 0; 0; 0]
            fx (ndarray): transition matrix
            fu (ndarray): transition matrix
            Xr (ndarray): estimate state of robot
            Xh (ndarray): estimate state of obstacle
            dot_Xr (ndarray): estimate gradient of robot state
            dot_Xh (ndarray): estimate gradient of obstacle state
            Mr (ndarray): cartesian position and velocity of the nearest point from robot to obstacle
            Mh (ndarray): cartesian position and velocity of the nearest point from obstacle to robot
            p_Mr_p_Xr (ndarray): derivative of robot cartesian state to internal state
            p_Mh_p_Xh (ndarray): derivative of obstacle cartesian state to internal state
            u0 (ndarray): reference control input, this is the default control input if there is no obstacle
            min_u (ndarray): control input saturation
            max_u (ndarray): control input saturation

        """
        
        return u0;