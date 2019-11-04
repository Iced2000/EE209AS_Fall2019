import numpy as np
import math
import random
import matplotlib.pyplot as plt

class Vertex():
    def __init__(self, state):
        self.state = state
        self.parent = None
    
    def findNearest(self, vertices):
        distMin = math.inf
        xnearest = Vertex((0,0,0))
        for v in vertices:
            dist = math.sqrt((self.state[0] - v.state[0])**2 + (self.state[1] - v.state[1])**2)
            if dist < distMin:
                distMin = dist
                xnearest = v
        return xnearest

class Robot():

    def __init__(self):
        self.w = 90
        self.l = 75             # tail length
        self.r = 25             # radius
        self.x = 0              # center x,y & orientation
        self.y = 0
        self.orientation = 0
        self.omegaMax = 2 * math.pi       # max wheel speed 1 revolution/s = 60 RPM = 2pi rad/s
    
    def setx(self, x):
        self.x = x
    
    def sety(self, y):
        self.y = y

    def setOrientation(self, o):
        self.orientation = o
    
    def setInit(self, init):
        self.x = init[0]
        self.y = init[1]
        self.orientation = init[2]
    
    def computeMove(self, target, k1, k2, k3, t):
        dx = target[0] - self.x
        dy = target[1] - self.y
        dr = math.sqrt(dx**2 + dy**2)
        if dr == 0 :
            print('stay still')
            return
        pi = math.pi
        angle1 = (math.atan2(dy, dx) - self.orientation + pi) % (2 * pi) - pi
        angle2 = (target[2] - self.orientation - angle1 + pi) % (2 * pi) - pi
        v = k1 * dr
        if angle1 > pi/2 or angle1 < - pi/2:
            v = -v 
        omega = k2 * angle1 + k3 * angle2
        # compute the control inputs
        omegaRight =  (2 * v + omega * self.w) / (2 * self.r)
        omegaLeft =  (2 * v - omega * self.w) / (2 * self.r)
        if max(abs(omegaRight), abs(omegaLeft)) > self.omegaMax:
            if abs(omegaRight) > abs(omegaLeft):
                omegaRight = omegaRight * self.omegaMax / abs(omegaRight)
                omegaLeft = omegaLeft * self.omegaMax / abs(omegaRight)
            else:
                omegaRight = omegaRight * self.omegaMax / abs(omegaLeft)
                omegaLeft = omegaLeft * self.omegaMax / abs(omegaLeft)
        v = (omegaLeft + omegaRight) * self.r / 2
        omega = (omegaRight - omegaLeft) * self.r / self.w
        # move and set new x,y,ori..
        self.orientation = self.orientation + omega * t
        self.x = self.x + v * math.cos(self.orientation) * t
        self.y = self.y + v * math.sin(self.orientation) * t
        
        return (self.x, self.y, self.orientation), (omegaLeft, omegaRight)

class Environment():

    def __init__(self, W, L):
        self.W = W
        self.L = L
        self.robot = Robot()
        self.obs = []                                           # list of obstacles in the form of (x, y, w, l)
        self.goalRegion = (0,0,0,0)
        # self.target = target
        self.target = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)
        while self.colDetect(self.target):
            # print('target has potential risk of collision, reset to random !')
            self.target = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)                                          # e.g. [3*W/4, 3*L/4]
        
        self.init = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)
        while self.colDetect(self.init):
            self.init = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)
        
        self.robot.setInit(self.init)

    def colDetect(self, state):
        # make sure four corners of the robot are safe.
        corners = np.zeros((4,2))
        theta1 = math.atan2(self.robot.w/2, 25)
        theta2 = math.atan2(self.robot.w/2, self.robot.l)
        rou1 = math.sqrt((self.robot.w/2)**2 + 25**2)
        rou2 = math.sqrt((self.robot.w/2)**2 + self.robot.l**2)
        corners[0]= [state[0] + rou1*math.cos(state[2]-theta1), state[1] + rou1*math.sin(state[2]-theta1)]
        corners[1]= [state[0] + rou1*math.cos(state[2]+theta1), state[1] + rou1*math.sin(state[2]+theta1)]
        corners[2]= [state[0] + rou2*math.cos(state[2]-theta2), state[1] + rou2*math.sin(state[2]-theta2)]
        corners[3]= [state[0] + rou2*math.cos(state[2]+theta2), state[1] + rou2*math.sin(state[2]+theta2)]
        for o in self.obs:
            for i in range(4):
                if abs(corners[i][0] - o[0] - o[2]/2) < o[2]/2 and abs(corners[i][1] - o[1] - o[3]/2) < o[3]/2 :
                    return True
        return False

    def setObs(self, obs):
        self.obs = obs

    def setGoalRegion(self, gr):
        self.goalRegion = gr

    def setTarget(self, target):
        self.target = target
        while self.colDetect(self.target):
            print('target state has potential risk of collision, reset to random !')
            self.target = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)

    def setInit(self, init):
        self.init = init
        while self.colDetect(self.init):
            print('initial state has potential risk of collision, reset to random !')
            self.init = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)
        self.robot.setInit(self.init)

    def trajGen(self, init, target, tf, dt, k1, k2, k3, plot):
        t = 0
        controlInputs = []
        traj = [init]
        self.robot.setInit(init)
        while t < tf :
            state, input = self.robot.computeMove(target, k1, k2, k3, dt)    # k1,k2,k3 needs to be modified later!!!
            controlInputs.append(input)
            traj.append(state)
            t = t + dt
        if plot :
            plt.figure()
            plt.arrow(init[0], init[1], np.cos(init[2]), np.sin(init[2]), color='r', width=1)
            plt.arrow(target[0], target[1], np.cos(target[2]), np.sin(target[2]), color='g', width=1)
            trajArray = np.reshape(traj, [len(traj), 3])
            plt.plot(trajArray[:, 0], trajArray[:, 1])
            plt.ylim([0, self.L])
            plt.xlim([0, self.W])
            plt.show()

        return controlInputs, traj

    def plotObs(self):
        fig = plt.figure()
        ax = fig.gca()
        for ob in self.obs:
            obstacle = plt.Rectangle(ob[0:2], ob[2], ob[3], color = 'k')
            ax.add_patch(obstacle)
        plt.xlim([0, self.W])
        plt.ylim([0, self.L])
        plt.show()

    def trajColFree(self, traj):
        tcf = True
        for state in traj:
            if self.colDetect(state):
                tcf = False
                break
        return tcf

    def rrt(self):                                                 
        xi = Vertex(self.init)
        vertices = [xi]

        fig = plt.figure()
        ax = fig.gca()
        for ob in self.obs:
            obstacle = plt.Rectangle(ob[0:2], ob[2], ob[3], color = 'k')
            ax.add_patch(obstacle)
        plt.xlim([0, self.W])
        plt.ylim([0, self.L])

        while 1 :
            xrand = Vertex((self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi))
            xnearest = xrand.findNearest(vertices)
            _, traj = self.trajGen(xnearest.state, xrand.state, 1, 0.1, 0.3, 1.5, -0.3, False)
            if self.trajColFree(traj):
                xnew = Vertex(traj[-1])
                xnew.parent = xnearest
                vertices.append(xnew)
                plt.plot([xnew.state[0], xnearest.state[0]], [xnew.state[1], xnearest.state[1]], color = 'g')
                if abs(xnew.state[0] - self.goalRegion[0] - self.goalRegion[2]/2) < self.goalRegion[2]/2 \
                and abs(xnew.state[1] - self.goalRegion[1] - self.goalRegion[3]/2) < self.goalRegion[3]/2:
                    break
        plt.show()
        traj = [xnew.state]
        x = xnew
        while x.parent != None:
            x = x.parent
            traj.append(x.state)
        traj.reverse()
        return  vertices, traj

# obs = []
# obs.append((5,5,1,1))
# obs.append((2,2,1,1))
# target = (8, 8, math.pi)
# e = Environment(10, 10)
# state, controlInputs = e.robot.computeMove(e.target, 0.3, 1.5, -0.3, 0.1)
# print('1')