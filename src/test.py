from agents import *
from models import *
from utils.World import *

# end class world

# instantiate the class
dT = 0.05
# robot = RobotArm(SublevelSafeSet(), dT);
# robot = Unicycle(SafeSet(), dT);
robot = Ball(SafeSet(), dT);
# human = InteractiveHumanBall2D(SafeSet(d_min=1, k_v=2), dT);
human = HumanBall2D(MobileAgent, dT)
w = World(dT, human, robot)
base.run()
