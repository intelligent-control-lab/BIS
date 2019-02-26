from agents import *
from models import *
from utils.World import *

# end class world

# instantiate the class
dT = 0.02
robot = RobotArm(PotentialField(), dT);
human = HumanBall3D(MobileAgent(), dT);
w = World(dT, human, robot)
base.run()
