import matplotlib.pyplot as plt
import numpy as np
from agents import *
from models import *
from utils.Tuner import Tuner
from matplotlib2tikz import save as tikz_save
from scipy.spatial import ConvexHull
from cycler import cycler
def roc_curve(models, settings):

    ret = dict()

    
    c = 0
    for model in models:

        ret[model] = dict()
        fig = plt.figure() 
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
            if not (sum(collision) == 0):
                first_safe = [i for i, e in enumerate(collision) if abs(e) > 1e-9][-1]+1
            if first_safe >= len(collision):
                print(args[0]+' params range is too narrow. Safe params set can not be acquired.')
                first_safe = len(collision)-1
            s = [20 * (abs(x) < 1e-9) for x in collision]
            auc = sum([(efficiency[i]+efficiency[i+1])*(safety[i+1]-safety[i])/2 for i in range(first_safe, len(collision)-1) ])
            print('Best param set:')
            print(param_set[first_safe])
            print('Best performance safety')
            print(safety[first_safe])
            print('Best performance efficiency')
            print(efficiency[first_safe])
            
            
            # line, = plt.plot(safety, efficiency, label=args[0])
            # plt.scatter(safety, efficiency, s =s, marker='o')
            # if args[0] == 'BarrierFunction':
            #     for i, txt in enumerate(param_set):
            #         plt.annotate(round(collision[i],2), (safety[i], efficiency[i]), size=5)
            safety = np.array(safety)
            efficiency = np.array(efficiency)


            plt.scatter(safety, efficiency, label=args[0])

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
            for i in range(len(hv)-1):
                k = p[hv[i+1], 0] - p[hv[i], 0]
                if k <= 0:
                    plt.plot(p[hv[i:i+2], 0], p[hv[i:i+2], 1], c='C'+str(c))

            c += 1
            ret[model][args[0]] = auc
            x = np.linspace(-20, -0.01, 100)

            # plt.plot(x, np.poly1d(np.polyfit(np.log(-safety + 1e-9), efficiency, 1))(np.log(-x)))
            #{'safety':safety[first_safe], 'efficiency':efficiency[first_safe]}
        plt.xlim(-2, 0)
        plt.ylim(0, 8)
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