
from direct.showbase.ShowBase import ShowBase
base = ShowBase()

from panda3d.core import *
from direct.task.Task import Task
from direct.gui.OnscreenText import OnscreenText
from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import *
import sys
from agents import *
from models import *
import numpy as np

def clamp(i, mn=-1, mx=1):
    return min(max(i, mn), mx)

class World(DirectObject):
    def __init__(self):
        # This is the initialization we had before
        globalClock.setMode(ClockObject.MForced);
        self.frame_rate = 50;
        globalClock.setFrameRate(self.frame_rate);
        print(ConfigVariableDouble("clock-frame-rate"))
        base.setFrameRateMeter(True)

        base.setBackgroundColor(0, 0, 0)  # Set the background to black
        base.disableMouse()  # disable mouse control of the camera
        camera.setPos(0, 0, 30)  # Set the camera position (X, Y, Z)
        camera.setHpr(0, -90, 0)  # Set the camera orientation
        #(heading, pitch, roll) in degrees


        self.sizescale = 1;

        self.map_size = 10;

        self.simRunning = True;

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
    
    def add_sphere(self, pos, color, scale=0.5):
        ret = loader.loadModel("resource/planet_sphere")
        ret.reparentTo(render)
        ret.setTransparency(TransparencyAttrib.MAlpha);
        ret.setColor(color[0], color[1], color[2], color[3]);
        ret.setScale(scale);
        ret.setPos(pos[0], pos[1], pos[2]);
        return ret;
    
    def draw_arrow(self, p_from, p_to, color):
        segs = LineSegs( )
        segs.setThickness( 20.0 )
        segs.setColor( color )
        segs.moveTo( p_from )
        segs.drawTo( p_to )
        arrow = segs.create( )
        render.attachNewNode(arrow)
        return segs
    
    def draw_movement(self, X, u):
        p_from = LVector3f(X[0], X[1], 0);
        v_to = p_from + LVector3f(X[2], X[3], 0);
        u_to = p_from + LVector3f(u[0], u[1], 0);
        u_color = Vec4(0.2, 0.8, 0.2, 0.5);
        v_color = Vec4(0.8, 0.2, 0.8, 0.5);
        return [self.draw_arrow(p_from, v_to, v_color), self.draw_arrow(p_from, u_to, u_color)];
    
    def draw_unicycle_control(self, X):
        p_from = LVector3f(X[0], X[1], 0);
        v_to = p_from + LVector3f(X[2], X[3], 0);
        v_color = Vec4(0.8, 0.2, 0.8, 0.5);
        return self.draw_arrow(p_from, v_to, v_color)
        
    def draw_half_plane(self, coef):
        segs = LineSegs( )
        segs.setThickness( 20.0 )
        segs.setColor( Vec4(0.1, 0.5, 0.8, 0.5) )
        A = coef[0];
        B = coef[1];
        C = coef[2];
        x0 = -10;
        x1 = 10;
        if B == 0:
            if A == 0:
                x0 = -100;
                x1 = -100;
            else:
                x0 = -C / A;
                x1 = -C / A;
            y0 = -10;
            y1 = 10;
        else:
            y0 = (-C - A * x0) / B;
            y1 = (-C - A * x1) / B;
        p_from = LVector3f(x0, y0, 0)
        p_to = LVector3f(x1, y1, 0);
        segs.moveTo( p_from )
        segs.drawTo( p_to )
        half_plane = segs.create( )
        # self.draw_square(Vec3(x0, y0, 0), Vec3(x1, y1, 0));

        return half_plane

    def add_unicycle(self, pos, color, scale=0.5):
        ret = loader.loadModel("resource/cube")
        ret.reparentTo(render)
        ret.setTransparency(TransparencyAttrib.MAlpha);
        ret.setColor(color[0], color[1], color[2], color[3]);
        ret.setScale(scale,0.1,scale);
        ret.setPos(pos[0], pos[1], pos[2]);
        return ret;

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

        self.dT = 1 / self.frame_rate;

        self.robot = Unicycle([-l,-l,0,0], SafeSet(), self.dT, True);
        self.agent_model = self.add_unicycle(list(self.robot.get_P()[:,0]) + [0],  [0.1, 0.5, 0.8, 1]);
        self.goal_model = self.add_sphere([self.robot.goal[0], self.robot.goal[1],0], [0.1, 0.5, 0.8, 0.5]);
        self.robot_v = self.draw_unicycle_control(list(self.robot.get_PV()[:,0])+[0]);
        # self.half_plane = self.draw_half_plane(self.robot.safe_set);
        # self.half_plane_handle = render.attachNewNode(self.half_plane);

        self.human = Human([l,l,0,0], MobileAgent(), self.dT, True);
        self.agent_model = self.add_sphere(list(self.human.get_P()[:,0])+[0], [0.8, 0.3, 0.2, 0.8]);
        self.goal_model = self.add_sphere([self.human.goal[0], self.human.goal[1],0], [0.8, 0.3, 0.2, 0.5]);
        [self.human_v, self.human_u] = self.draw_movement(list(self.human.get_PV()[:,0])+[0], list(self.human.u[:,0])+[0]);

    
    def move_models(self, task):
        # Check to make sure the mouse is readable
        if base.mouseWatcherNode.hasMouse():
            # get the mouse position as a LVector2. The values for each axis are from -1 to
            # 1. The top-left is (-1,-1), the bottom right is (1,1)
            mpos = base.mouseWatcherNode.getMouse()
            # Here we multiply the values to get the amount of degrees to turn
            # Restrain is used to make sure the values returned by getMouse are in the
            # valid range. If this particular model were to turn more than this,
            # significant tearing would be visable
            self.human.update(self.robot);
            self.human.move(mpos.getX()*5, mpos.getY()*5);
            self.agent_model.setPos(self.human.get_P()[0], self.human.get_P()[1], 0);
            self.goal_model.setPos(self.human.goal[0], self.human.goal[1], 0);
            self.move_seg(self.human_u, list(self.human.get_P()[:,0])+[0], list(self.human.u[:,0])+[0])
            self.move_seg(self.human_v, list(self.human.get_P()[:,0])+[0], list(self.human.get_V()[:,0])+[0])

            self.robot.update(self.human);
            # print('robot hpr')
            # print(self.agent_model.getHpr())
            # print(self.robot.x)
            self.robot.move();
            self.agent_model.setPos(self.robot.get_P()[0], self.robot.get_P()[1], 0);
            self.agent_model.setH(self.robot.x[3,0] / np.pi * 180);
            # print('robot hpr')
            # print(self.agent_model.getHpr())
            # print('set pos')
            # print(list(self.robot.get_P()[:,0])[0], list(self.robot.get_P()[:,0])[1])
            self.goal_model.setPos(self.robot.goal[0], self.robot.goal[1], 0);
            
            self.move_seg(self.robot_v, list(self.robot.get_P()[:,0])+[0], list(self.robot.get_V()[:,0])+[0])

            # print('--')
        return Task.cont  # Task continues infinitely
    # end load_models()
    def move_seg(self, vdata, p_from, vec):
        p_from = LVector3f(p_from[0], p_from[1], p_from[2])
        p_to = p_from + LVector3f(vec[0], vec[1], vec[2])
        vdata.setVertex(0, p_from)
        vdata.setVertex(1, p_to)
    def test_half(self, parent, coef):
        segs = LineSegs( )
        segs.setThickness( 20.0 )
        segs.setColor( Vec4(0.1, 0.5, 0.8, 0.5) )
        A = coef[0];
        B = coef[1];
        C = coef[2];
        for i in range(-5,5):
            for j in range(-5,5):
                if A*i+B*j+C < 0: 
                    ball = loader.loadModel("resource/planet_sphere")
                    ball.setTransparency(TransparencyAttrib.MAlpha);
                    ball.setColor(0.1, 0.9, 0.1, 0.3);
                    ball.setScale(0.1);
                    ball.setPos(i,j,0);
                    ball.reparentTo(parent)

    def setupLights(self):  # Sets up some default lighting
        ambientLight = AmbientLight("ambientLight")
        ambientLight.setColor((.4, .4, .35, 1))
        directionalLight = DirectionalLight("directionalLight")
        directionalLight.setDirection(LVector3(0, 8, -2.5))
        directionalLight.setColor((0.9, 0.8, 0.9, 1))
        render.setLight(render.attachNewNode(directionalLight))
        render.setLight(render.attachNewNode(ambientLight))

# end class world

# instantiate the class
w = World()
base.run()
