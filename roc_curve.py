import matplotlib.pyplot as plt
import numpy as np
from agents import *
from models import *
from utils.Tuner import Tuner
from matplotlib2tikz import save as tikz_save
from scipy.spatial import ConvexHull
from cycler import cycler
import shutil, os
def roc_curve(models, settings):

    ret = dict()

    
    c = 0
    
    for model in models:

        ret[model] = dict()
        fig = plt.figure() 
        his = []
        hie = []
        for args in settings:
            
            print('===============')
            print(args)
            tuner = Tuner(model ,args[0], args[1])
            result = tuner.tune()
            result.sort(key=lambda tup: tup[0])

            print('result')
            print(*result, sep = "\n")

            safety, efficiency, collision, param_set = tuple(map(list, zip(*result)))
            
            first_safe = 0
            collision = np.array(collision)
            safety = np.array(safety)
            efficiency = np.array(efficiency)
            if not (sum(collision) == 0):
                first_safe = [i for i, e in enumerate(collision) if abs(e) > 1e-9][-1]+1

            nc_idx = np.where(collision < 5e-2)[0]
            if len(nc_idx) == 0:
                print(args[0]+' params range is too narrow. Safe params set can not be acquired.')

            else:
                
                hi = nc_idx[np.argmax(efficiency[nc_idx])]
                print('Hybrid param set:')
                print(param_set[hi])
                print('Hybrid performance safety')
                print(safety[hi])
                print('Hybrid performance efficiency')
                print(efficiency[hi])
                plt.scatter(safety[hi:hi+1], efficiency[hi:hi+1], c='C'+str(c), marker='P', s=200, zorder=10)
                
                print(os.path.join('eval_results', args[0], param_set[hi][:-2]))
                csrc = os.path.join('eval_results', model, args[0], param_set[hi][:-2])
                cdst = os.path.join('Hybrid_score_result', model, args[0]+'_'+param_set[hi][:-2])
                if os.path.exists(cdst):
                    shutil.rmtree(cdst)
                shutil.copytree(csrc, cdst)

                
            s = [20 * (abs(x) < 1e-9) for x in collision]
            auc = sum([(efficiency[i]+efficiency[i+1])*(safety[i+1]-safety[i])/2 for i in range(first_safe, len(collision)-1) ])

            
            # print(nc_idx)
            # print(efficiency[nc_idx])
            # print(np.argmax(efficiency[nc_idx]))
            
            
            
            # line, = plt.plot(safety, efficiency, label=args[0])
            # plt.scatter(safety, efficiency, s =s, marker='o')
            # if args[0] == 'BarrierFunction':
            #     for i, txt in enumerate(param_set):
            #         plt.annotate(round(collision[i],2), (safety[i], efficiency[i]), size=5)
            


            p = np.vstack([safety, efficiency]).T
            
            hull = ConvexHull(p)
            print('hull.simplices')
            print(hull.simplices)

            def calc_k(p):
                if (p[1,0] - p[0,0]) == 0:
                    return 1e9 if p[1,1] > p[0,1] else -1e9
                return (p[1,1] - p[0,1]) / (p[1,0] - p[0,0])
            hv = hull.vertices
            hv = np.append(hv, hv[0])

            idx = []
            for i in range(len(hv)-1):
                k = p[hv[i+1], 0] - p[hv[i], 0]
                if k <= 0:
                    plt.plot(p[hv[i:i+2], 0], p[hv[i:i+2], 1], c='C'+str(c), linewidth=3)
                    idx.append(hv[i])
                    idx.append(hv[i+1])

            plt.scatter(safety[idx], efficiency[idx], label=args[0], c='C'+str(c), s=20)
            mask = np.ones(len(safety))
            mask[idx] = 0
            rest = np.where(mask)[0]
            plt.scatter(safety[rest], efficiency[rest], c='C'+str(c), alpha=.2, s=20, linewidth=0)
            

            c += 1
            ret[model][args[0]] = auc
            x = np.linspace(-20, -0.01, 100)

            # plt.plot(x, np.poly1d(np.polyfit(np.log(-safety + 1e-9), efficiency, 1))(np.log(-x)))
            #{'safety':safety[first_safe], 'efficiency':efficiency[first_safe]}
        plt.xlim(-20, 0)


        plt.ylim(0, 10)
        fig.legend()
        plt.xlabel('Safety')
        plt.ylabel('Efficiency')
        # tikz_save(model+'.tex')
        fig.savefig(model+'.pdf', bbox_inches='tight')
        # plt.show()

    return ret

if __name__ == "__main__":
    models = ['Ball3D']
    settings = [ \
        ('SafeSet',          {'d_min': np.arange(0, 5, 1.0), 'yita': np.arange(0,10,2.0)}),\
        ('BarrierFunction',  {'d_min': np.arange(0, 5, 1.0), 't': np.arange(0.2,2,0.4)}),\
        ('PotentialField',   {'d_min': np.arange(0, 5, 1.0), 'lambd': np.arange(2,10,2)}),\
        ('SlidingMode',      {'d_min': np.arange(0, 5, 1.0)}),\
    ];
    roc_curve(models, settings)