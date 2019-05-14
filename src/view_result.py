from agents import *
from models import *
from utils.World import *
import sys
import pickle
# end class world

# instantiate the class
f = open(sys.argv[1], 'rb')
record = pickle.load(f)
dT = record.dT
exec('robot = ' + record.model + '(' + record.algorithm + '(), dT)');
human = HumanBall3D(MobileAgent(), dT);

w = World(dT, human, robot, record)
base.run()
