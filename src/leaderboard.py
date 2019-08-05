from roc_curve import roc_curve
import numpy as np
import matplotlib
# matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
def leaderboard():
    """
    This function call evaluate() function to test algorithms on given parameter ranges. Parameter ranges are related to models. The parameters will be grid searched for each algorithms, and generate the roc curve by a convex hull to cover all results on the safety-efficiency plot.
    """
    # models = ['Ball3D']
    # settings = [ \
    #     # ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     # ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('PotentialField',          {'d_min': [1, 2, 3], 'k_v': [0.5, 1, 2], 'c1': [1, 3, 5]}),\
    # ]
    # roc_curve(models, settings, True)

    # models = ['Unicycle']
    # settings = [ \
    #     # ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     # ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('PotentialField',          {'d_min': [1, 2, 3], 'k_v': [0.5, 1, 2], 'c1': [1, 3, 5]}),\
    # ]
    # roc_curve(models, settings, True)

    models = ['SCARA']
    settings = [ \
        ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
        ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
        ('SublevelSafeSet',  {'d_min': [1, 2, 3],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        ('ZeroingBarrierFunction',              {'d_min': [2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 2, 5, 10]}),\
        ('PotentialField',              {'d_min': [1, 2, 3], 'k_v': [0.5, 1, 2], 'c1': [1, 3, 5]}),\
    ]
    roc_curve(models, settings, True)

    # models = ['RobotArm']
    # settings = [ \
    #     # ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     # ('SafeSublevelSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     # ('SublevelSafeSet',  {'d_min': [1, 2, 3],  'k_v': [0.25, 0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10, 20, 50, 100, 200]}),\
    #     ('PotentialField',          {'d_min': [1, 2, 3], 'k_v': [0.5, 1, 2], 'c1': [1, 3, 5]}),\
    # ]
    # roc_curve(models, settings, True)

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

    # models = ['Ball3D']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('SafeSublevelSet',  {'d_min': [1, 2, 3],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[0.5, 1, 2]}),\
    #     ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # roc_curve(models, settings, False)

    # models = ['Unicycle']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # roc_curve(models, settings, False)

    # models = ['SCARA']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # roc_curve(models, settings, False)

    # models = ['RobotArm']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [0.1, 0.2, 0.3, 1, 2, 3]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('SafeSublevelSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('ZeroingBarrierFunction',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # roc_curve(models, settings, False)

if __name__ == "__main__":
    leaderboard()