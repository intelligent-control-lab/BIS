from agents import *
from models import *
from utils.ParallelWorld import *
import sys
import pickle
# end class world

# instantiate the class
robots = []
records = []
for i in range(1,len(sys.argv)):
    f = open(sys.argv[i], 'rb')
    record = pickle.load(f)
    dT = record.dT
    records.append(record)
    exec('robots.append(' + record.model + '(' + record.algorithm + '(), dT))');
human = HumanBall3D(MobileAgent(), dT);

w = ParallelWorld(dT, human, robots, records)
base.run()
