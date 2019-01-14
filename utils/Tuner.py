from agents import *
from models import *
from evaluate import evaluate
# 

class Tuner():

    def __init__(self, model, algorithm, params_dict):
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
        
    def dfs(self, idx, param_str, param_set):
        
        if idx == len(self.params):
            score = evaluate(self.model, self.algorithm, False, self.robot, param_str[:-2])
            self.safety.append(-score['safety'])
            self.efficiency.append(score['efficiency'])
            self.collision_cnt.append(score['collision_cnt'])
            self.result.append((-score['safety'], score['efficiency'], score['collision_cnt'], param_str))
            return

        for p in self.params_range[idx]:
            p = round(p, 3)
            exec('self.robot.agent.'+self.params[idx]+' = '+str(p))
            param_set[self.params[idx]] = p
            self.dfs(idx+1, param_str+self.params[idx]+'='+str(p)+'__', param_set)

    def tune(self):
        self.dfs(0, '', dict())
        return self.result
