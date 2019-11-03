import numpy as np
import math
import random

class Robot():

    def __init__(self):
        self.w = 90
        self.l = 75             # tail length
        self.r = 25             # radius
        self.x = 0              # center x,y & orientation
        self.y = 0
        self.orientation = 0
        self.leftSpeed = 0
        self.rightSpeed = 0
        self.maxSpeed = 2 * math.pi * self.r       # wheel speed 1 revolution/s = 60 RPM
    
    def setLeftSpeed(self, speed):
        self.leftSpeed = speed
    
    def setRightSpeed(self, speed):
        self.rightSpeed = speed
    
    def setx(self, x):
        self.x = x
    
    def sety(self, y):
        self.y = y
    
    def computeMove(self, x, y):
        dx = x - self.x
        dy = y - self.y
        dr = math.sqrt(pow(dx, 2) + pow(dy, 2))
        if dr == 0 :
            print('stay still')
            return
        pi = math.pi
        angle = math.atan2(dy, dx) - self.orientation
        angle = angle % (2 * pi)
        forward = True
        if angle > pi / 2 and angle < pi * 3 / 2:   # rotate less and move backward 
            angle = angle - pi
            forward = False
        elif angle > pi * 3 / 2 :
            angle = angle - 2 * pi

        # if angle == 0:
        #     self.leftSpeed = 0
        #     self.rightSpeed = 0
        # if angle > 0:
        #     self.leftSpeed = -self.maxSpeed
        #     self.rightSpeed = self.maxSpeed
        # else:
        #     self.leftSpeed = self.maxSpeed
        #     self.rightSpeed = -self.maxSpeed

        # timeRotate = (abs(angle) * self.w / 2) / self.maxSpeed

        # if forward :
        #     self.leftSpeed = self.maxSpeed
        #     self.rightSpeed = self.maxSpeed
        # else:
        #     self.leftSpeed = self.-maxSpeed
        #     self.rightSpeed = self.-maxSpeed
        
        # timeTranslate = dr / self.maxSpeed

        return dr, angle, forward

class Environment():

    def __init__(self, W, L, obs, target):
        self.W = W
        self.L = L
        self.robot = Robot()
        self.obs = obs                                                  # list of obstacles in the form of (x, y, w, l)
        
        self.target = target
        while self.colDetect(self.target[0], self.target[1]):
            print('target has potential risk of collision, reset to random !')
            self.target = (self.W * random.random(), self.L * random.random())                                          # e.g. [3*W/4, 3*L/4]
        
        self.init = (self.W * random.random(), self.L * random.random())
        while self.colDetect(self.init[0], self.init[1]):
            self.init = (self.W * random.random(), self.L * random.random())
        
        self.robot.setx(self.init[0])
        self.robot.sety(self.init[1])


    def colDetect(self, x, y):
        for o in self.obs:
            if abs(x - o[0]) < o[2]/2 and abs(y - o[1]) < o[3]/2 :
                return True
        return False

    def setObs(self, obs):
        self.obs = obs

    def setTarget(self, target):
        self.target = target
        while self.colDetect(self.target[0], self.target[1]):
            print('target has potential risk of collision, reset to random !')
            self.target = (self.W * random.random(), self.L * random.random())

    def genTraj(self):                                                  # should include RRT here and get a traj
        traj = []
        return traj

obs = []
obs.append((5,5,1,1))
obs.append((2,2,1,1))
target = (8, 8)
e = Environment(10, 10, obs, target)
dr, angle, forward = e.robot.computeMove(e.target[0], e.target[1])
print('1')