import sys, glob, os
import pickle
from agents import *
from models import *
from utils.Record import Record

def save_data(folder, prefix, record):
    """
    This function save the generated data to disk.
    """
    cnt = len(glob.glob1(folder,prefix+"*"))
    f = open(os.path.join(folder, prefix+str(cnt)), 'wb')
    print(os.path.join(folder, prefix+str(cnt)))
    pickle.dump(record, f)

def generate_date(period, amount):
    """
    This function generates random data for given models.
    """
    fps = 20
    dT = 1 / fps
    model_name = ['Ball3D', 'RobotArm', 'SCARA', 'Unicycle']

    human_start = [10, 0, 10, 0]
    robot_start = [1e5, 0, 1e5, 0]

    for k in range(amount):
        robot = RobotArm(SafeSet(), dT)
        human = HumanBall3D(MobileAgent(), dT, True)
        record = Record(period, dT, human.goals, robot.goals, human.x, robot.x)
        for i in range(fps * period):
            human.update(robot)
            human.move()
            if record.cnt < record.period * fps:
                record.human_moves[:, record.cnt] = human.x
                record.cnt = record.cnt + 1
        
        record.human_achieved = human.goal_achieved
        save_data('simulate_data', 'data_', record)

if __name__ == '__main__':
    period = int(sys.argv[1])
    amount = int(sys.argv[2])
    generate_date(period, amount)