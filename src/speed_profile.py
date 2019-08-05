from agents import *
from models import *
import numpy as np
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import sys
import pickle
# end class world

def speed_profile(file_names):
    """
    This function is to plot speed profiles for several evaluation results.
    Args:
        file_names (array of string): file names to be draw speed profile.
    """

    # instantiate the class
    robots = []
    records = []
    dT = 0.05
    for i in range(1,len(file_names)):
        f = open(file_names[i], 'rb')
        record = pickle.load(f)
        records.append(record)
        exec('robots.append(' + record.model + '(' + record.algorithm + '(), dT))');
        
    print(len(records))

    fig = plt.figure() 
    ax1=plt.subplot(2, 1, 1)
    ax2=plt.subplot(2, 1, 2)

    for i in range(len(records)):
        d = []
        dot_d = []
        human = HumanBall3D(MobileAgent(), dT);
        for t in range(records[0].tot):
            records[i].robot_moves[:, t]

            human.update(robots[0])
            human.move(records[0].human_moves[:, t])

            robots[i].update(human)
            robots[i].x = records[i].robot_moves[:, t]

            Mr = robots[i].m
            Mh = human.m

            dim = np.shape(Mr)[0] // 2
            p_idx = np.arange(dim)
            v_idx = p_idx + dim

            d.append(np.linalg.norm(Mr[p_idx] - Mh[p_idx]))
            sgn = (Mr[p_idx+dim] - Mh[p_idx+dim]).T * (Mr[p_idx] - Mh[p_idx])
            sgn = -1 if sgn < 0 else 1
            dot_d.append(sgn * np.linalg.norm(Mr[p_idx+dim] - Mh[p_idx+dim]))
        print(d[:10])
        print(dot_d[:10])

        
        ax1.plot(d, c='C'+str(i), label=records[i].algorithm, linestyle='-')
        
        ax2.plot(dot_d, c='C'+str(i), label=records[i].algorithm, linestyle='--')
        ax2.plot(range(-100,800,100), np.linspace(0,0,9),c='black', linestyle='-')
        

        
    ax1.legend()
    ax1.set_xlim(0,200)
    ax1.set_ylabel('m', fontsize = 20)
    # plt.show()
    # fig.legend()

    ax2.set_xlim(0,200)
    ax2.set_xlabel('Frame (0.05s)', fontsize = 20)
    ax2.set_ylabel('m/s', fontsize = 20)
    # tikz_save(model+'.tex')
    fig.savefig('speed_profile.pdf', bbox_inches='tight')

if __name__ == '__main__':
    speed_profile(sys.argv)
    