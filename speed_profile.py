from agents import *
from models import *
import numpy as np
import matplotlib.pyplot as plt
import sys
import pickle
# end class world

# instantiate the class
robots = []
records = []
dT = 0.05
for i in range(1,len(sys.argv)):
    f = open(sys.argv[i], 'rb')
    record = pickle.load(f)
    records.append(record)
    exec('robots.append(' + record.model + '(' + record.algorithm + '(), dT))');
    
print(len(records))
fig = plt.figure() 
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

    plt.plot(d, c='C'+str(i), label=records[i].algorithm+'_dis', linestyle='-')
    plt.plot(dot_d, c='C'+str(i), label=records[i].algorithm+'_vel', linestyle='--')
    plt.plot(range(-100,800,100), np.linspace(0,0,9),c='b', linestyle='-')
    plt.xlim(0,200)
plt.legend()    
plt.show()