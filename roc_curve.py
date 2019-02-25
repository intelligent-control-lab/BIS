import matplotlib.pyplot as plt
import numpy as np
from agents import *
from models import *
from utils.Tuner import Tuner
from matplotlib2tikz import save as tikz_save

def roc_curve(models, settings):

    ret = dict()
    for model in models:

        ret[model] = dict()
        fig = plt.figure() 
        for args in settings:
            print('===============')
            print(args)
            tuner = Tuner(model ,args[0], args[1])
            result = tuner.tune()
            print('result')
            print(result)
            result.sort(key=lambda tup: tup[0])
            safety, efficiency, collision, param_set = tuple(map(list, zip(*result)))
            safety = safety / np.max(safety)
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
            
            line, = plt.plot(safety, efficiency, label=args[0])
            # plt.scatter(safety, efficiency, s =s, marker='o')
            # if args[0] == 'BarrierFunction':
            #     for i, txt in enumerate(param_set):
            #         plt.annotate(round(collision[i],2), (safety[i], efficiency[i]), size=5)
            ret[model][args[0]] = auc
            #{'safety':safety[first_safe], 'efficiency':efficiency[first_safe]}
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