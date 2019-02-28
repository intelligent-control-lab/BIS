from agents import *
from models import *
from utils.World import *

# end class world

# instantiate the class
dT = 0.05
robot = SCARA(SafeSet(), dT);
human = HumanBall2D(MobileAgent(), dT);
w = World(dT, human, robot)
base.run()
