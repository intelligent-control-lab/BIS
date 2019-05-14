
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

class ParallelWorld(DirectObject):
    def __init__(self, dT, human, robots, records = None):

        self.human = human
        self.robots = robots

        blue = [0.1, 0.5, 0.8, 0.8]
        green = [0.1, 0.8, 0.3, 0.8]
        purple = [0.7, 0.2, 0.6, 0.8]
        self.color_strings = ['blue', 'green', 'purple']

        self.colors = [blue, green, purple]
        
        self.records = records
        
        if not self.records is None:
            self.human.reset(records[0 ].dT, records[0].human_goals)
            for i in range(len(self.robots)):
                self.robots[i].reset(records[i].dT, records[i].robot_goals)
        
        # This is the initialization we had before
        globalClock.setMode(ClockObject.MForced);
        self.frame_rate = int(1/dT);
        self.dT = dT
        globalClock.setFrameRate(self.frame_rate);
        print(ConfigVariableDouble("clock-frame-rate"))
        base.setFrameRateMeter(True)

        base.setBackgroundColor(0, 0, 0)  # Set the background to black
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

        self.sky = loader.loadModel("resource/solar_sky_sphere")
        self.sky.reparentTo(render)
        self.sky.setScale(10)
        self.sky_tex = loader.loadTexture("resource/stars_1k_tex.jpg")
        self.sky.setTexture(self.sky_tex, 1)

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
        for i in range(len(self.robots)):
            textObject = OnscreenText(text = self.color_strings[i] + ' -> ' + self.records[i].algorithm, fg = self.colors[i], pos = (-1, 0.7 - 0.05 * i), scale = 0.05)
            self.robots[i].load_model(render, loader, self.colors[i])
        
        # self.human = HumanBall3D(MobileAgent(), self.dT, True, [l,l,l,0,0,0]);
        self.human.load_model(render, loader)
        
        if self.auxiliary:
            self.human.model_auxiliary()
            for i in range(len(self.robots)):
                self.robots[i].model_auxiliary()
        
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
            dv = 30
            self.camera_h_v = dv if mx > 0.5 else -dv if mx < -0.5 else 0
            self.camera_p_v = -dv if my > 0.5 else dv if my < -0.5 else 0


            self.camera_h = self.camera_h + self.camera_h_v * self.dT
            self.camera_p = self.camera_p + self.camera_p_v * self.dT

            r = 40
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


            if self.records is None:
                self.human.update(self.robots[0])
                self.human.move()
                self.human.redraw_model()
                
                for i in range(len(self.robots)):
                    self.robots[i].update(self.human)
                    self.robots[i].move()
                    self.robots[i].redraw_model()
            else:
                self.human.update(self.robots[0])
                self.human.move(self.records[0].human_moves[:, self.cnt])
                self.human.redraw_model()
                self.human.draw_trace()
                
                for i in range(len(self.robots)):
                    print(i, ' ', self.robots[i].x)
                    self.robots[i].update(self.human)
                    self.robots[i].x = self.records[i].robot_moves[:, self.cnt]
                    self.robots[i].redraw_model()
                    self.robots[i].draw_trace()
                
                self.cnt = self.cnt + 1
                if self.cnt >= self.records[0].tot:
                     sys.exit()
                    # taskMgr.remove("move_models")

            
            if self.auxiliary:
                self.human.model_auxiliary()
                for i in range(len(self.robots)):
                    self.robots[i].model_auxiliary()    

            # if self.robot.agent.fuck:
            #     taskMgr.remove("move_models")

            
            # print('--')
        return Task.cont  # Task continues infinitely
    # end load_models()


    def setupLights(self):  # Sets up some default lighting
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.4, .4, .35, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(LVector3(0, 8, -2.5))
        directionalLight.setColor((0.9, 0.8, 0.9, 1))
        render.setLight(render.attachNewNode(directionalLight))
        render.setLight(render.attachNewNode(ambientLight))

