import pickle
import os, sys, glob
import numpy as np
from models import *
from agents import *
# from utils.World import World

def evaluate(model, algorithm, graphics = False, robot = None, save_postfix=None, passive = True):
    """This function evaluate an algorithm's performance on a given model. To avoid repeated test, the algorithm will examine whether the results already exist. If they do, it will return the results directly. Which also means if you updated some algorithm, you need to delete previous results manually to see the changes.

    Args:
        model (str): the name of the model to be tested
        algorithm (str): the name of the algorithm to be tested
        graphics (bool): whether to use graphics to show the evaluation process on live
        robot (KinematicModel): you can pass in an initialized agent with a given setting, or the function will use a default one. This is useful when you grid search the parameters.
        save_postfix (str): a string to specify the name of the results
        passive(bool): whether the human model is passive to the robot's behavior. If is ture, human model will not react to the robot.

    Returns:
        total_score (dict): A dict contains the algorithm's average score on different aspects, include safety, efficiency, collision count, and nearest distance.

    """


    
    if save_postfix is None:
        save_postfix = 'best'
    
    save_dir = os.path.join('eval_results', model, algorithm, save_postfix)
    if not passive:
        save_dir = os.path.join('interactive_eval_results', model, algorithm, save_postfix)
    
    # Avoid repetition, which also means if you updated some algorithm, you need to delete previous results manually to see the changes.

    if robot is None:
        robot = eval(model + '(' + algorithm + '(), dT)')
    
        
    if glob.glob1(save_dir, 'total_score'):
        f = open(os.path.join(save_dir, 'total_score'), 'rb')
        total_score = pickle.load(f)
        # print('total_score')
        # print(total_score)
        return total_score
    
    dT = 0.02
    
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    data_dir = 'simulate_data'
    names = glob.glob1(data_dir, 'data_*')
    
    total_score = dict()
    
    n = len(names)
    for name in names:
        # print(name)
        f = open(os.path.join(data_dir, name), 'rb')

        record = pickle.load(f)
        record.robot_moves = np.matrix(np.zeros((np.shape(robot.x)[0], record.tot)))
        record.cnt = 0

        if passive:
            if robot.is_2D:
                human = HumanBall2D(MobileAgent(), dT)
            else:
                human = HumanBall3D(MobileAgent(), dT)
        else:
            if robot.is_2D:
                human = InteractiveHumanBall2D(SafeSet(d_min=1, k_v=1), dT)
            else:
                human = InteractiveHumanBall3D(SafeSet(d_min=1, k_v=1), dT)


        #Make sure all the algorithms have same goal sequence
        human.reset(record.dT, record.human_goals)
        robot.reset(record.dT, record.robot_goals)
        
        score = dict()

        record.model = model
        record.algorithm = algorithm

        for t in range(record.tot):
            human.update(robot)
            if passive:
                human.move(*record.human_moves[:,t])
            else:
                human.move()
                if np.shape(human.x)[0] == 4:
                    record.human_moves[:, t] = np.vstack([human.x[[0,1]], 0, human.x[[2,3]], 0])
                else:
                    record.human_moves[:, t] = human.x

            robot.update(human)
            robot.move()
            record.robot_moves[:, t] = robot.x

        # if graphics:
            
        #     try:
        #         w = World(record.dT, human, robot, record)
        #         base.run()
        #     except SystemExit as e:
        #         pass

        save_data(save_dir, name.replace('data', 'result'), record)
        for k in robot.score.keys():
            if k not in total_score:
                total_score[k] = robot.score[k] / n
            else:
                total_score[k] = total_score[k] + robot.score[k] / n

        if not passive:
            total_score['efficiency'] += human.score['efficiency'] / n

        # print('score[efficiency]')
        # print(robot.score['efficiency'])
        # print('score[collision_cnt]')
        # print(robot.score['collision_cnt'])
    # print('total_score')
    # print(total_score)
    print('total_score[efficiency]')
    print(total_score['efficiency'])
    print('total_score[collision_cnt]')
    print(total_score['collision_cnt'])
    save_data(save_dir, 'total_score', total_score)
    return total_score

def save_data(folder, name, record):
    """
    This function saves the results.

    Args:
        folder: folder path
        name: file name
        record: evaluation result
    """
    if not os.path.exists(folder):
        os.makedirs(folder)
    f = open(os.path.join(folder, name), 'wb')
    print(os.path.join(folder, name))
    pickle.dump(record, f)
    
if __name__ == "__main__":
    graphics = False
    evaluate(sys.argv[1], sys.argv[2], graphics=graphics)