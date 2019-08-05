from agents import *
from models import *
from utils.ParallelWorld import *
import sys
import pickle
# end class world

def visual_compare(file_names):
    """
    This script is to visualize evaluation results simultaneously.

    Args:
        file_names (array of string): file names to be draw speed profile.
    """
    # instantiate the class
    robots = []
    records = []
    for i in range(1,len(file_names)):
        f = open(file_names[i], 'rb')
        record = pickle.load(f)
        dT = record.dT
        records.append(record)
        exec('robots.append(' + record.model + '(' + record.algorithm + '(), dT))');
    human = HumanBall3D(MobileAgent(), dT);

    w = ParallelWorld(dT, human, robots, records)
    base.run()

if __name__ == '__main__':
    visual_compare(sys.argv)