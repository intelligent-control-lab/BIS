from roc_curve import roc_curve
import numpy as np
import matplotlib.pyplot as plt
def leaderboard():
    """
    This function generates roc curves for given models based on given parameter ranges
    """
    models = ['Ball3D']
    settings = [ \
        ('BarrierFunction',  {'d_min': [1, 2, 3],  't':[0.5, 0.75, 1, 1.5, 2], 'gamma':[1, 2, 3, 5]}),\
        ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
        ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
        ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
        ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        # ('SafeSublevelSet',  {'d_min': [1, 2, 3],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[0.5, 1, 2]}),\
    ]

    # models = ['Unicycle']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': [1, 2, 3],  't':[0.5, 0.75, 1, 1.5, 2], 'gamma':[1, 2, 3, 5]}),\
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    # ]

    # models = ['SCARA']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    # ]


    # models = ['RobotArm']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [0.1, 0.2, 0.3, 1, 2, 3]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    # ]

    ret = roc_curve(models, settings)
    # fig, axs =plt.subplots(len(models)+1,1)
    # for i,model in enumerate(models):
    #     print(i, model)
    #     algs = list(ret[model].keys())
    #     # safe = [x['safety'] for x in ret[model].values()]
    #     # effi = [x['efficiency'] for x in ret[model].values()]
    #     auc = list(ret[model].values())
    #     table = np.vstack([algs, auc]).T
    #     collabel=("Algorithm", "AUC")
    #     the_table = axs[i].table(cellText=table,colLabels=collabel,loc='center')
    # plt.show()

if __name__ == "__main__":
    leaderboard()