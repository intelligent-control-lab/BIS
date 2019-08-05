from agents import *
from models import *
import copy
from evaluate import evaluate
import multiprocessing as mp
from itertools import product
# 

class Tuner():

    def __init__(self, model, algorithm, params_dict, passive=True):
        self.safety = []
        self.efficiency = []
        self.collision_cnt = []
        self.result = []
        self.model = model
        self.algorithm = algorithm
        self.params = list(params_dict.keys())
        self.params_range = list(params_dict.values())
        #dT will be reset when evaluating by simulate_data
        self.robot = eval(model + '(' + algorithm + '(), 0.02)')
        self.param_combs = list()
        self.robots = list()
        self.passive = passive
        
    def dfs(self, idx, param_str, param_set, robot):
        
        if idx == len(self.params):
            self.param_combs.append(param_str)
            self.robots.append(robot)
            return

        for p in self.params_range[idx]:
            p = round(p, 3)
            param_set[self.params[idx]] = p
            new_robot = copy.deepcopy(robot)
            exec('new_robot.agent.'+self.params[idx]+' = '+str(p))
            self.dfs(idx+1, param_str+self.params[idx]+'='+str(p)+'__', param_set, new_robot)
    
    def processInput(self, param_str, robot):
            score = evaluate(self.model, self.algorithm, False, robot, param_str[:-2], self.passive)
            # self.safety.append(-score['safety'])
            # self.efficiency.append(score['efficiency'])
            # self.collision_cnt.append(score['collision_cnt'])
            # print(-score['safety'])
            return (score['safety'], score['efficiency'], score['collision_cnt'], param_str)
            
 
    def tune(self):
        self.dfs(0, '', dict(), self.robot)
        num_cores = mp.cpu_count()
        pool = mp.Pool(num_cores)
        self.result = pool.starmap(self.processInput, zip(self.param_combs, self.robots))
        return self.result
