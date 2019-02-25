from roc_curve import roc_curve
import numpy as np
import matplotlib.pyplot as plt
def leaderboard():
    """
    This function generates roc curves for given models based on given parameter ranges
    """
    models = ['Unicycle']
    settings = [ \
        ('BarrierFunction',  {'d_min': np.arange(2, 3, 1)}),\
        ('SafeSet',          {'d_min': np.arange(2, 3, 1)}),\
        ('PotentialField',   {'d_min': np.arange(2, 3, 1)}),\
        ('SlidingMode',      {'d_min': np.arange(2, 3, 1)}),\
    ]

    # models = ['SCARA','RobotArm']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': np.arange(0, 5, 1.0), 't': np.arange(0.6, 3.0, 0.4)}),\
    #     ('SafeSet',          {'d_min': np.arange(0, 5, 1.0), 'yita': np.arange(3, 10, 2.0)}),\
    #     ('PotentialField',   {'d_min': np.arange(0, 5, 1.0), 'lambd': np.arange(0.2, 3, 0.2)}),\
    #     # ('PotentialField',   {'d_min': np.arange(0, 5, 0.5), 'lambd': np.arange(1, 20, 2.0)}),\
    #     ('SlidingMode',      {'d_min': np.arange(0, 5, 1.0)}),\
    # ];

    # models = ['Ball3D','Unicycle']
    # settings = [ \
    #     ('BarrierFunction',  {'d_min': np.arange(0, 5, 0.5), 't': np.arange(0.2, 2.0, 0.2)}),\
    #     ('SafeSet',          {'d_min': np.arange(0, 5, 0.5), 'yita': np.arange(1 , 10, 1.0)}),\
    #     ('PotentialField',   {'d_min': np.arange(0, 5, 0.5), 'lambd': np.arange(1, 20, 2.0)}),\
    #     ('SlidingMode',      {'d_min': np.arange(0, 5, 0.5)}),\
    # ]

    # models = ['Unicycle']#, 'SCARA', 'RobotArm']
    # settings = [ \
    #     ('SafeSet',          {'d_min': np.arange(0, 5, 1.0), 'yita': np.arange(2 ,10, 2.0)}),\
    #     ('BarrierFunction',  {'d_min': np.arange(0, 5, 1.0), 't': np.arange(0.4, 2.0, 0.4)}),\
    #     ('PotentialField',   {'d_min': np.arange(0, 5, 1.0), 'lambd': np.arange(2,10,2.0)}),\
    #     ('SlidingMode',      {'d_min': np.arange(0, 5, 1.0)}),\
    # ];

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