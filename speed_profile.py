from agents import *
from models import *
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
    
for t in range(records[0].tot):
    for i in range(records):
        records[i].robot_moves[:, t]
        
        self.robots[i].update(self.human)
        self.robots[i].x = self.records[i].robot_moves[:, self.cnt]