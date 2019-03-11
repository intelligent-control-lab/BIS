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
        ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
        ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    ]
    ret = roc_curve(models, settings, True)

    models = ['Unicycle']
    settings = [ \
        ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
        ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
        ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
        ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        # ('BarrierFunction',  {'d_min': [1, 2, 3, 4, 5],  't':[0.5, 0.75, 1, 1.5, 2], 'gamma':[0.1, 1, 2, 3, 5]}),\
        ('BarrierFunction',  {'d_min': [4, 5, 7, 10],  't':[0.5, 1, 2], 'gamma':[0.1, 1, 2, 5]}),\
        ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
        ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    ]
    ret = roc_curve(models, settings, True)

    models = ['SCARA']
    settings = [ \
        ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
        ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
        ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
        ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
        ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
        ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    ]
    ret = roc_curve(models, settings, True)

    models = ['RobotArm']
    settings = [ \
        ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
        ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [0.1, 0.2, 0.3, 1, 2, 3]}),\
        ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
        ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        ('SafeSublevelSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
        ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
        ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
        ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    ]
    ret = roc_curve(models, settings, True)

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
    #     ('BarrierFunction',  {'d_min': [1, 2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.1, 1, 2, 5]}),\
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('SafeSublevelSet',  {'d_min': [1, 2, 3],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[0.5, 1, 2]}),\
    #     ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # ret = roc_curve(models, settings, False)

    # models = ['Unicycle']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('BarrierFunction',  {'d_min': [1, 2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.1, 1, 2, 5]}),\
    #     ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # ret = roc_curve(models, settings, False)

    # models = ['SCARA']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [3, 5, 10, 15, 20]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # ret = roc_curve(models, settings, False)

    # models = ['RobotArm']
    # settings = [ \
    #     ('SlidingMode',      {'d_min': [1, 1.5, 2, 2.5, 3], 'k_v': [1, 1.5, 2], 'u_p': [1, 5, 10]}),\
    #     ('PotentialField',   {'d_min': [1, 1.5, 2, 2.5, 3], 'lambd': [0.1, 0.2, 0.3, 1, 2, 3]}),\
    #     ('SafeSet',          {'d_min': [1, 1.5, 2, 2.5, 3],  'yita': [1, 2, 4, 8], 'k_v': [1, 1.5, 2]}),\
    #     ('SublevelSafeSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     # ('SafeSublevelSet',  {'d_min': [1, 2],  'k_v': [0.5, 1, 1.5, 2], 'gamma':[1, 2, 5, 10]}),\
    #     ('BarrierFunction',  {'d_min': [2, 3, 4, 5],  't':[0.5, 1, 2], 'gamma':[0.5, 1, 2, 3]}),\
    #     ('ZBF',              {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    #     ('ZBF_as',           {'d_min': [1, 2, 3, 4],  't':[0.5, 1, 2, 5], 'gamma':[0.1, 1, 5]}),\
    # ]
    # ret = roc_curve(models, settings, False)

if __name__ == "__main__":
    leaderboard()