import numpy as np
from numpy import zeros, matrix
class Record:

    def __init__(self, period, dT, human_goals, robot_goals, human_state, robot_state):
        self.model = ''
        self.algorithm = ''
        
        self.dT = dT
        self.period = period
        self.cnt = 0
        self.tot = int(period / dT)

        self.mouse_pos = matrix(zeros((2, self.tot)))

        n = np.shape(human_state)[0]
        self.human_moves = matrix(zeros((n, self.tot)))
        self.human_goals = human_goals
        self.human_achieved = 0

        m = np.shape(robot_state)[0]
        self.robot_moves = matrix(zeros((m, self.tot)))
        self.robot_goals = robot_goals
        self.robot_achieved = 0
    


    
