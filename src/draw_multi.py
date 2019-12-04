import matplotlib.pyplot as plt
import numpy as np
from agents import *
from models import *
from utils.Tuner import Tuner
from scipy.spatial import ConvexHull
from cycler import cycler
import shutil, os
import pickle

plt.rcParams["font.family"] = "Times New Roman"

x_lims = [-25, -30, -50, -30]
models = ['Ball3D', 'Unicycle', 'SCARA','RobotArm']
algorithms  = ['ZBF', 'PFM', 'SlidingMode', 'SafeSet',  'SublevelSafeSet']
labels = {  'PotentialField': 'PFM',
            'SafeSet':'SSA',
            'ZBF':'BFM',
            'PFM':'PFM',
            'SlidingMode':'SMA',
            'SublevelSafeSet':'SSS' }
lg_lb = list(map(lambda x:labels[x], algorithms))
passive = True
m = 0

fig = plt.figure(figsize=(25, 4))
ax = fig.subplots(ncols=4, )
hd = []
for model in models:
    c = 0
    
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
        param_set = np.array(param_set)
        if not (sum(collision) == 0):
            first_safe = [i for i, e in enumerate(collision) if abs(e) > 1e-9][-1]+1

        nc_idx = np.where(collision < 5e-2)[0]
        if len(nc_idx) == 0:
            print(algo+' params range is too narrow. Safe params set can not be acquired.')
        else:
            hi = nc_idx[np.argmax(efficiency[nc_idx])]
            # print(algo, efficiency[hi], 'col=',collision[hi])
            ax[m].scatter(safety[hi:hi+1], efficiency[hi:hi+1], c='C'+str(c), marker='P', s=200, zorder=10)
            
        s = [20 * (abs(x) < 1e-9) for x in collision]
        auc = sum([(efficiency[i]+efficiency[i+1])*(safety[i+1]-safety[i])/2 for i in range(first_safe, len(collision)-1) ])

        p = np.vstack([safety, efficiency]).T
        
        hull = ConvexHull(p)

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
            ax[m].plot(upperx, uppery, c='C'+str(c),  label=labels[algo], linewidth=3)
            
#======================================
        else:
#============ convex bound ============
            first = True
            for i in range(len(hv)-1):
                k = p[hv[i+1], 0] - p[hv[i], 0]
                if  k < 0 :
                    hh = ax[m].plot(p[hv[i:i+2], 0], p[hv[i:i+2], 1], c='C'+str(c), linewidth=3)
                    idx.append(hv[i])
                    idx.append(hv[i+1])

                    if m == 3 and first:
                        first = False
                        hd.append(hh)
                        print(m, c, len(hd), i)

            ax[m].scatter(safety[idx], efficiency[idx], label=labels[algo], c='C'+str(c), s=20)
#======================================
        mask = np.ones(len(safety))
        mask[idx] = 0
        rest = np.where(mask)[0]
        ax[m].scatter(safety[rest], efficiency[rest], c='C'+str(c), alpha=.2, s=20, linewidth=0)
        c += 1            

    if passive:
        # plt.xlim(x_lims[m], 1)
        ax[m].set_xlim(x_lims[m])
    m += 1

fig.subplots_adjust(left = 0.07, right = 0.95, bottom=0.25)

print(lg_lb)
leg = fig.legend(hd,     # The line objects
           labels=lg_lb,   # The labels for each line
           loc="upper center",   # Position of legend
           borderaxespad=0.1,    # Small spacing around legend box
        #    title=""  # Title for the legend
           ncol=5,
           fontsize=12,
           )
leg.legendHandles[0].set_color('C0')
leg.legendHandles[1].set_color('C1')
leg.legendHandles[2].set_color('C2')
leg.legendHandles[3].set_color('C3')
leg.legendHandles[4].set_color('C4')
fig.text(0.15, 0.02, 'Ball', ha='center', fontsize=12)
fig.text(0.38, 0.02, 'Unicycle', ha='center', fontsize=12)
fig.text(0.62, 0.02, 'SCARA', ha='center', fontsize=12)
fig.text(0.85, 0.02, 'Robot Arm', ha='center', fontsize=12)

fig.text(0.5, 0.1, 'Safety', ha='center', fontsize=12)
fig.text(0.02, 0.5, 'Efficiency', va='center', rotation='vertical', fontsize=12)
plt.show()

save_name = '_multi.pdf'
if not passive:
    save_name = 'interactive_results' + save_name
else:
    save_name = 'passive_results' + save_name
fig.savefig(save_name, bbox_inches='tight')