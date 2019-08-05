from agents import *
from models import *
from utils.World import *
import sys
import pickle
"""
This script is to visualize evaluation result.

usage:
python view_result.py path_to_record_file
"""
# instantiate the class
f = open(sys.argv[1], 'rb')
record = pickle.load(f)
dT = record.dT
exec('robot = ' + record.model + '(' + record.algorithm + '(), dT)');
human = HumanBall3D(MobileAgent(), dT);

w = World(dT, human, robot, record)
base.run()
