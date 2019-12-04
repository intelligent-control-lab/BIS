
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from direct.task.Task import Task
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import *
import sys
from agents import *
from models import *
import numpy as np
from numpy import sin, cos, pi


base = ShowBase()

class World(DirectObject):
    def __init__(self, dT, human, robot, record = None):

        self.human = human
        self.robot = robot
        
        self.record = record
        
        if not self.record is None:
            self.human.reset(record.dT, record.human_goals)
            self.robot.reset(record.dT, record.robot_goals)
        
        # This is the initialization we had before
        globalClock.setMode(ClockObject.MForced);
        self.frame_rate = int(1/dT);
        self.dT = dT
        globalClock.setFrameRate(self.frame_rate);
        print(ConfigVariableDouble("clock-frame-rate"))
        base.setFrameRateMeter(True)


        base.setBackgroundColor(1., 1., 1.)  # Set the background to black
        base.disableMouse()  # disable mouse control of the camera

        
        self.camera_h = 0
        self.camera_p = -60

        self.sizescale = 1;
        self.map_size = 10;
        self.simRunning = True;

        self.auxiliary = False

        # Now that we have finished basic initialization, we call load_models which
        # will handle actually getting our objects in the world
        self.load_models()
        self.setupLights()

        taskMgr.add(self.move_models, "move_models")

        self.mouse1EventText = self.genLabelText(
            "Mouse Button 1: Pause or resume game", 1)

        self.accept("mouse1", self.handleMouseClick)

    def genLabelText(self, text, i):
        return OnscreenText(text=text, pos=(0.06, -.06 * (i + 0.5)), fg=(1, 1, 1, 1),
                            parent=base.a2dTopLeft,align=TextNode.ALeft, scale=.05)

    def handleMouseClick(self):
        if self.simRunning:
            taskMgr.remove('move_models')
            
        else:
            taskMgr.add(self.move_models, "move_models")
        # toggle self.simRunning
        self.simRunning = not self.simRunning
    # end handleMouseClick

    def load_models(self):

        l = self.map_size/2;

        self.cube = loader.loadModel("resource/cube")
        self.cube.setColor(0.5,0.5,0.5)
        self.cube.setScale(l, l, 0.001);
        self.cube.setPos(0, 0, -0.5);
        self.cube.reparentTo(render)

        # self.dT = 1 / self.frame_rate;
        # self.robot = Ball(self.init_state, SafeSet(), self.dT , True);
        # self.robot = Unicycle(self.init_state,  BarrierFunction(), self.dT , True);
        # 
        # self.robot = SCARA(self.init_state, SlidingMode(), self.dT , True);
        
        # self.robot = RobotArm(SafeSet(), self.dT);
        self.robot.load_model(render, loader)
        
        # self.human = HumanBall3D(MobileAgent(), self.dT, True, [l,l,l,0,0,0]);
        self.human.load_model(render, loader)
        
        if self.auxiliary:
            self.human.model_auxiliary()
            self.robot.model_auxiliary()
        
        self.cnt = 0

    def move_models(self, task):
        # Check to make sure the mouse is readable

        if base.mouseWatcherNode.hasMouse():
            # get the mouse position as a LVector2. The values for each axis are from -1 to
            # 1. The top-left is (-1,-1), the bottom right is (1,1)
            mw = base.mouseWatcherNode
            mx, my = mw.getMouseX(), mw.getMouseY()
            # Here we multiply the values to get the amount of degrees to turn
            # Restrain is used to make sure the values returned by getMouse are in the
            # valid range. If this particular model were to turn more than this,
            # significant tearing would be visable
            dv = 0
            self.camera_h_v = dv if mx > 0.5 else -dv if mx < -0.5 else 0
            self.camera_p_v = -dv if my > 0.5 else dv if my < -0.5 else 0

            self.camera_h = self.camera_h + self.camera_h_v * self.dT
            self.camera_p = self.camera_p + self.camera_p_v * self.dT

            r = 20
            z = r * sin(abs(self.camera_p / 180 * pi))
            xy = r * cos(abs(self.camera_p / 180 * pi))
            x = xy * sin(self.camera_h / 180 * pi)
            y = -xy * cos(self.camera_h / 180 * pi)

            # print('x, y, z')
            # print(x, y, z)
            # print('h, p')
            # print(self.camera_h, self.camera_p)
            camera.setPos(x, y, z)  # Set the camera position (X, Y, Z)
            camera.setHpr(self.camera_h, self.camera_p, 0)  # Set the camera orientation
            # camera.setPos(0, -20, 30)  # Set the camera position (X, Y, Z)
            # camera.setHpr(self.camera_h, self.camera_p, 0)  # Set the camera orientation
            #(heading, pitch, roll) in degrees


            if self.record is None:
                self.human.update(self.robot)
                self.human.move()
                self.human.redraw_model()
                
                self.robot.update(self.human)
                self.robot.move()
                self.robot.redraw_model()
            else:
                self.human.update(self.robot)
                self.human.move(self.record.human_moves[:, self.cnt])
                self.human.redraw_model()
                
                self.robot.update(self.human)
                self.robot.x = self.record.robot_moves[:, self.cnt]
                self.robot.redraw_model()
                
                self.cnt = self.cnt + 1
                if self.cnt >= self.record.tot:
                     sys.exit()
                    # taskMgr.remove("move_models")

            
            if self.auxiliary:
                self.human.model_auxiliary()
                self.robot.model_auxiliary()    

            if self.robot.agent.fuck:
                taskMgr.remove("move_models")

            
            # print('--')
        return Task.cont  # Task continues infinitely
    # end load_models()


    def setupLights(self):  # Sets up some default lighting
        
        self.light = render.attachNewNode(Spotlight("Spot"))
        self.light.node().setScene(render)
        self.light.node().setShadowCaster(True, 4096, 4096)
        # self.light.node().showFrustum()
        self.light.node().getLens().setFov(40)
        self.light.setPos(0, -10, 40)
        self.light.lookAt(0, 0, 0)

        self.light.node().setCameraMask(BitMask32.bit(0))
        self.human.goal_model.hide(BitMask32.bit(0))
        self.robot.goal_model.hide(BitMask32.bit(0))
        
        self.light.node().getLens().setNearFar(1, 100)
        render.setLight(self.light)

        self.alight = render.attachNewNode(AmbientLight("Ambient"))
        self.alight.node().setColor(LVector4(0.4, 0.4, 0.4, 1))
        render.setLight(self.alight)

        render.find('**/agent').setColor(LVector4(0.4, 0, 0.4, 1))

        self.alight = render.find('**/agent').attachNewNode(AmbientLight("Ambient"))
        self.alight.node().setColor(LVector4(0.02, 0.05, 0.1, 0))
        render.find('**/agent').setLight(self.alight)

        # ylight = render.find('**/agent').attachNewNode(Spotlight("Spot"))
        # ylight.node().setScene(render)
        # ylight.node().setShadowCaster(True, 2048, 2048)
        # ylight.node().setColor(LVector4(0.8, 0.4, 0.0, 1))
        # # ylight.node().showFrustum()
        # ylight.node().getLens().setFov(40)
        # ylight.setPos(0, 10, 20)
        # ylight.lookAt(0, 0, 0)
        # render.find('**/agent').setLight(ylight)


        # Important! Enable the shader generator.
        render.setShaderAuto()