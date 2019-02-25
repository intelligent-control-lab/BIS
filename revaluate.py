import os, sys, glob
import pickle
import numpy as np
from models import *
from agents import *
from utils.World import World

def evaluate(model, algorithm, graphics = False, robot = None, save_postfix=None):
    """This function evaluate a algorithm's performance on a given model.

    Args:
        param1 (int): The first parameter.
        param2 (str): The second parameter.
        model (str): a string for the model name
        algorithm (str): a string for the algorithm name
        graphics (bool): whether to use graphics to show the results
        robot (KinematicModel): you can pass in a initialized agent with a given setting. Or the function will give a default one. This is useful when you grid search the parameters.
        save_postfix (str): a string 

    Returns:
        total_score (dict): A dict contains the algorithm's average score on different aspects.

    """


    
    if save_postfix is None:
        save_postfix = 'best'
    save_dir = os.path.join('eval_results', model, algorithm, save_postfix)
    
    # Avoid repetition, which also means if you updated some algorithm, you need to delete former results manually to see the changes.
    
    
    dT = 0.02
    
    if robot is None:
        robot = eval(model + '(' + algorithm + '(), dT)')
    
    
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    data_dir = 'simulate_data'
    names = glob.glob1(data_dir, 'data_*')
    
    total_score = dict()
    
    n = len(names)
    for name in names:
        # print(name)
        save_path = os.path.join(save_dir, name.replace('data', 'result'))
        if os.path.exists(save_path):
            f = open(save_path, 'rb')
            record = pickle.load(f)

        else:

        f = open(os.path.join(data_dir, name), 'rb')

        record = pickle.load(f)
        record.robot_moves = np.matrix(np.zeros((np.shape(robot.x)[0], record.tot)))
        record.cnt = 0
        if robot.is_2D:
            human = HumanBall2D(MobileAgent(), dT)
        else:
            human = HumanBall3D(MobileAgent(), dT)

        #Make sure all the algorithms have same goal sequence
        human.reset(record.dT, record.human_goals)
        robot.reset(record.dT, record.robot_goals)
        
        score = dict()

        record.model = model
        record.algorithm = algorithm

        for t in range(record.tot):
            human.update(robot)
            human.move(*record.human_moves[:,t])
            robot.update(human)
            robot.move()
            record.robot_moves[:, t] = robot.x

            record.robot_closest_P[:, t] = robot.m
            record.human_closest_P[:, t] = human.m

        if graphics:
            
            try:
                w = World(record.dT, human, robot, record)
                base.run()
            except SystemExit as e:
                pass


        save_data(save_dir, name.replace('data', 'result'), record)
        for k in robot.score.keys():
            if k not in total_score:
                total_score[k] = robot.score[k] / n
            else:
                total_score[k] = total_score[k] + robot.score[k] / n

        print('score[efficiency]')
        print(robot.score['efficiency'])
        print('score[collision_cnt]')
        print(robot.score['collision_cnt'])
    # print('total_score')
    # print(total_score)
    print('total_score[efficiency]')
    print(total_score['efficiency'])
    print('total_score[collision_cnt]')
    print(total_score['collision_cnt'])
    save_data(save_dir, 'total_score', total_score)
    return total_score

def save_data(folder, name, record):
    if not os.path.exists(folder):
        os.makedirs(folder)
    f = open(os.path.join(folder, name), 'wb')
    print(os.path.join(folder, name))
    pickle.dump(record, f)
    
if __name__ == "__main__":
    graphics = False
    evaluate(sys.argv[1], sys.argv[2], graphics=graphics)