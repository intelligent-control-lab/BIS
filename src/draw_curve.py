import matplotlib
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from agents import *
from models import *
from utils.Tuner import Tuner
# from tikzplotlib import save as tikz_save
from scipy.spatial import ConvexHull
from cycler import cycler
import shutil, os
import pickle

def draw_curve(passive=True):
    """
    This function draws trade-off curves.

    Args:
        passive (bool, optional): Defaults to True. Whether to draw curves for passive human model results.
    """
    x_lims = [-25, -30, -50, -30]
    models = ['Ball3D', 'Unicycle', 'SCARA', 'RobotArm']
    # algorithms  = ['BarrierFunction', 'PotentialField', 'SlidingMode', 'SafeSet',  'SublevelSafeSet']
    algorithms  = ['ZBF', 'PFM', 'SlidingMode', 'SafeSet', 'SublevelSafeSet']
    labels = {  'PFM': 'Potential Field',
                'SafeSet':'Safe Set',
                'ZBF':'Barrier Function',
                'SlidingMode':'Sliding Mode',
                'SublevelSafeSet':'Sublevel Safe Set' }
    m = 0
    for model in models:
        c = 0
        fig = plt.figure()
        print('=====================')
        print(model)
        for algo in algorithms:
            
            cpf = open('drawing_data/'+model+'_'+algo+'_'+str(passive),'rb')
            cp = pickle.load(cpf)
            safety = cp['safety']
            efficiency = cp['efficiency']
            collision = cp['collision']
            param_set = cp['param_set']

            first_safe = 0
            collision = np.array(collision)
            safety = np.array(safety)
            efficiency = np.array(efficiency)
            if not (sum(collision) == 0):
                first_safe = [i for i, e in enumerate(collision) if abs(e) > 1e-9][-1]+1

            nc_idx = np.where(collision < 5e-2)[0]
            if len(nc_idx) == 0:
                print(algo+' params range is too narrow. Safe params set can not bee acquired.')
            else:
                hi = nc_idx[np.argmax(efficiency[nc_idx])]
                # print('Hybrid param set,  safety,  efficiency')
                # print(param_set[hi], safety[hi], efficiency[hi])
                # print('C'+str(c))
                print(algo, efficiency[hi], 'col=',collision[hi])
                plt.scatter(safety[hi:hi+1], efficiency[hi:hi+1], c='C'+str(c), marker='P', s=200, zorder=10)
                
            s = [20 * (abs(x) < 1e-9) for x in collision]
            auc = sum([(efficiency[i]+efficiency[i+1])*(safety[i+1]-safety[i])/2 for i in range(first_safe, len(collision)-1) ])

            p = np.vstack([safety, efficiency]).T
            
            hull = ConvexHull(p)

            print(*zip(safety, efficiency, collision, param_set))

            def calc_k(p):
                if (p[1,0] - p[0,0]) == 0:
                    return 1e9 if p[1,1] > p[0,1] else -1e9
                return (p[1,1] - p[0,1]) / (p[1,0] - p[0,0])
            hv = hull.vertices
            hv = np.append(hv, hv[0])

            idx = []
            if not passive:
    #============ upper bound ============
                lx = np.min(safety)
                rx = np.max(safety)
                steps = 5
                dx = (rx - lx) / steps
                upperx = []
                uppery = []
                for i in range(steps+1):
                    lb = lx + i*dx - dx/2
                    rb = lx + i*dx + dx/2
                    inrange = np.where((safety >= lb) & (safety <= rb))[0]
                    if(len(inrange) == 0):
                        continue
                    upperx.append(lx+i*dx)
                    uppery.append(np.max(efficiency[np.where((safety >= lb) & (safety <= rb))[0]]))
                plt.plot(upperx, uppery, c='C'+str(c),  label=labels[algo], linewidth=3)
    #======================================
            else:
    #============ convex bound ============
                for i in range(len(hv)-1):
                    k = p[hv[i+1], 0] - p[hv[i], 0]
                    if  k < 0 :
                        plt.plot(p[hv[i:i+2], 0], p[hv[i:i+2], 1], c='C'+str(c), linewidth=3)
                        idx.append(hv[i])
                        idx.append(hv[i+1])
                plt.scatter(safety[idx], efficiency[idx], label=labels[algo], c='C'+str(c), s=20)
    #======================================
            mask = np.ones(len(safety))
            mask[idx] = 0
            rest = np.where(mask)[0]
            plt.scatter(safety[rest], efficiency[rest], c='C'+str(c), alpha=.2, s=20, linewidth=0)
            
            c += 1            
            # x = np.linspace(-20, -0.01, 100)
            # plt.plot(x, np.poly1d(np.polyfit(np.log(-safety + 1e-9), efficiency, 1))(np.log(-x)))
            #{'safety':safety[first_safe], 'efficiency':efficiency[first_safe]}
        if passive:
            plt.xlim(x_lims[m], 1)
        m += 1


        # plt.ylim(0, 10)
        fig.legend(fontsize=12)
        plt.xlabel('Safety', fontsize=20)
        plt.ylabel('Efficiency', fontsize=20)
        # tikz_save(model+'.tex')
        save_name = model+'.pdf'
        if not passive:
            save_name = 'interactive_' + save_name
        else:
            save_name = 'passive_' + save_name

        fig.savefig('results/'+save_name, bbox_inches='tight')
        # plt.show()

if __name__ == '__main__':
    draw_curve()
    