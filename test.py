from agents import *
from models import *
from utils.World import *

# end class world

# instantiate the class
dT = 0.02
robot = Unicycle(PotentialField(), dT);
human = HumanBall2D(MobileAgent(), dT, auto = False);
w = World(dT, human, robot)
base.run()
