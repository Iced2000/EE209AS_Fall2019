import numpy as np
import math
import random
import matplotlib.pyplot as plt
from operator import itemgetter

def cross(p1, p2, p3):
    x1 = p2[0] - p1[0]
    y1 = p2[1] - p1[1]
    x2 = p3[0] - p1[0]
    y2 = p3[1] - p1[1]
    return  x1 * y2 - x2 * y1    

def segment(p1, p2, p3, p4): 
    if max(p1[0], p2[0]) >= min(p3[0], p4[0]) and max(p3[0], p4[0])>=min(p1[0], p2[0]) \
        and max(p1[1], p2[1]) >= min(p3[1], p4[1]) and max(p3[1], p4[1]) >= min(p1[1], p2[1]):
        if cross(p1, p2, p3) * cross(p1, p2, p4) <= 0 \
            and cross(p3, p4, p1) * cross(p3, p4, p2) <= 0 :
            return True
        else:
            return False
    else: 
        return False

class Tree(object):
    def __init__(self, vertices):
        self.v = vertices
        self.root = None

    def addVertex(self, v, root=False):
        #print("add", v)
        self.v.append(v)
        if root:
            self.root = v

    def findNearest(self, v1):
        distMin = math.inf
        for v in self.v:
            dist = v.dist(v1)
            if dist < distMin:
                distMin = dist
                vNearest = v
        return vNearest

    def pathCost(self, v2):
        pcost = 0
        while v2.parent is not None:
            cost, p = v2.parent
            pcost += cost
            v2 = p
        return pcost

    def getNearVertices(self, new):
        nV = []
        for v in self.v:
            nV.append((self.pathCost(v) + v.dist(new), v))
        nV.sort(key=itemgetter(0))
        return nV


class Vertex():
    def __init__(self, state):
        self.state = np.array(state)
        self.parent = None

    def setParent(self, p):
        self.parent = (p.dist(self), p)

    def dist(self, v):
        return np.linalg.norm(self.state - v.state)


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
        self.tree = Tree([])
        self.obs = []  # list of obstacles in the form of (x, y, w, l)
        self.goalRegion = (0,0,0,0)
        # self.target = target
        self.target = self.genRandomState()
        self.init = self.genRandomState()
        self.robot.setInit(self.init)
    
    def genRandomState(self):
        while True:
            rs = (self.W * random.random(), self.L * random.random(), 2 * math.pi * random.random() - math.pi)
            if not self.colDetect(rs):
                return rs

    def state2corners(self, state):
        corners = np.zeros((5,2))
        theta1 = math.atan2(self.robot.w/2, 25)
        theta2 = math.atan2(self.robot.w/2, self.robot.l)
        rou1 = math.sqrt((self.robot.w/2)**2 + 25**2)
        rou2 = math.sqrt((self.robot.w/2)**2 + self.robot.l**2)
        corners[0] = [state[0] + rou1*math.cos(state[2]-theta1), state[1] + rou1*math.sin(state[2]-theta1)]
        corners[1] = [state[0] + rou1*math.cos(state[2]+theta1), state[1] + rou1*math.sin(state[2]+theta1)]
        corners[2] = [state[0] + rou2*math.cos(state[2]-theta2-math.pi), state[1] + rou2*math.sin(state[2]-theta2-math.pi)]
        corners[3] = [state[0] + rou2*math.cos(state[2]+theta2-math.pi), state[1] + rou2*math.sin(state[2]+theta2-math.pi)]
        corners[4] = corners[0]
        return corners

    def colDetect(self, state):
        corners = self.state2corners(state)
        for o in self.obs:
            # make sure four corners of the robot are outside the square.
            for i in range(4):
                if abs(corners[i][0] - o[0] - o[2]/2) < o[2]/2 and abs(corners[i][1] - o[1] - o[3]/2) < o[3]/2 :
                    return True
            # make sure none of the edge of the robot intersects the diagonal of the obstacle square.
            p1 = [o[0], o[1]]
            p2 = [o[0], o[1] + o[3]]
            p3 = [o[0] + o[2], o[1]]
            p4 = [o[0] + o[2], o[1] + o[3]]
            for i in range(4):
                if segment(corners[i], corners[i+1], p1, p4) or segment(corners[i], corners[i+1], p2, p3):
                    return True

        return False

    def stateColDetect(self, state1, state2):

        # traj = [state1]
        # self.robot.setInit(state1)
        # while np.linalg.norm(state2[0:2] - (self.robot.x, self.robot.y)) > 1 :
        #     state, _ = self.robot.computeMove(state2, 0.3, 1.5, -0.3, 0.1)    # k1,k2,k3 needs to be modified later!!!
        #     traj.append(state)
        # return not self.trajColFree(traj)

        for o in self.obs:
            p1 = [o[0], o[1]]
            p2 = [o[0], o[1] + o[3]]
            p3 = [o[0] + o[2], o[1]]
            p4 = [o[0] + o[2], o[1] + o[3]]
            corners1 = self.state2corners(state1)
            corners2 = self.state2corners(state2)
            for i in range(4):
                if segment(corners1[i], corners2[i], p1, p4) or segment(corners1[i], corners2[i], p2, p3):
                    return True
        return False


    def setObs(self, obs):
        self.obs = obs

    def setGoalRegion(self, gr):
        self.goalRegion = gr

    def setTarget(self, target):
        self.target = target
        if self.colDetect(self.target):
            print('target state has potential risk of collision, reset to random !')
            self.target = self.genRandomState()
            
    def setInit(self, init):
        self.init = init
        if self.colDetect(self.init):
            print('initial state has potential risk of collision, reset to random !')
            self.init = self.genRandomState()
        self.robot.setInit(self.init)

    def trajGen(self, init, target, tf, dt, k1, k2, k3, plot = 0):
        t = 0
        controlInputs = []
        traj = [init]
        self.robot.setInit(init)
        while t < tf :
            state, input = self.robot.computeMove(target, k1, k2, k3, dt)    # k1,k2,k3 needs to be modified later!!!
            controlInputs.append(input)
            traj.append(state)
            t += dt
        if plot :
            plt.figure()
            plt.arrow(init[0], init[1], np.cos(init[2]), np.sin(init[2]), color='r', width=5)
            plt.arrow(target[0], target[1], np.cos(target[2]), np.sin(target[2]), color='g', width=5)
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

    def genNewAndNearest(self):
        while True:
            xrand = Vertex(self.genRandomState())
            xnearest = self.tree.findNearest(xrand)
            _, traj = self.trajGen(xnearest.state, xrand.state, 1, 0.1, 0.3, 1.5, -0.3, False)
            if self.trajColFree(traj) and traj[-1][0] > 0 \
                and traj[-1][0] < self.W and traj[-1][1] > 0 and traj[-1][1] < self.L:
                return Vertex(traj[-1]), xnearest

    def reconstructTree(self, nearVertices, xnew):
        for _, v in nearVertices:
            if v == self.tree.root:
                continue
            ori = self.tree.pathCost(v)
            tmp = self.tree.pathCost(xnew) + xnew.dist(v)
            if tmp < ori and not self.stateColDetect(v.state, xnew.state): #???????
                v.setParent(xnew)

    def checkGoal(self, xnew):
        return abs(xnew.state[0] - self.goalRegion[0] - self.goalRegion[2]/2) < self.goalRegion[2]/2 \
                and abs(xnew.state[1] - self.goalRegion[1] - self.goalRegion[3]/2) < self.goalRegion[3]/2

    def rrt(self):
        self.tree.addVertex(Vertex(self.init), root=True)
        i = 0
        while True:
            print(i)
            i += 1
            xnew, xnearest = self.genNewAndNearest()
            xnew.setParent(xnearest)
            self.tree.addVertex(xnew)
            if self.checkGoal(xnew):
                break
        """
        traj = [xnew.state]
        x = xnew
        while x.parent != None:
            _, x = x.parent
            traj.append(x.state)
        traj.reverse()
        """
        return self.tree, xnew

    def rrt_star(self):            
        self.tree.addVertex(Vertex(self.init), root=True)
        i = 0
        while True:
            print(i)
            i += 1
            xnew, _ = self.genNewAndNearest()
            nearVertices = self.tree.getNearVertices(xnew)

            for _, nV in nearVertices:
                if not self.stateColDetect(nV.state, xnew.state): #???????
                    xnew.setParent(nV)
                    self.tree.addVertex(xnew)
                    break

            self.reconstructTree(nearVertices, xnew)
            if self.checkGoal(xnew):
                break
        return self.tree, xnew

    def plotTree(self, x = None):
        fig = plt.figure()
        ax = fig.gca()
        for ob in self.obs:
            obstacle = plt.Rectangle(ob[0:2], ob[2], ob[3], color = 'k')
            ax.add_patch(obstacle)
        goalRegion = plt.Rectangle(self.goalRegion[0:2], self.goalRegion[2], self.goalRegion[3], color = 'y')
        ax.add_patch(goalRegion)
        plt.xlim([0-10, self.W+10])
        plt.ylim([0-10, self.L+10])

        for v in self.tree.v:
            if v != self.tree.root:
                x1, y1 = v.state[0], v.state[1]
                _ , p = v.parent
                x2, y2 = p.state[0], p.state[1]
                plt.plot([x1, x2], [y1, y2], color = 'g')
        if x != None:
            traj = []
            while x.parent != None:
                _, x = x.parent
                traj.append(x.state)
            traj.reverse()
            trajArray = np.reshape(traj, [len(traj), 3])
            plt.plot(trajArray[:, 0], trajArray[:, 1], color = 'r')
            plt.arrow(self.init[0], self.init[1], np.cos(self.init[2]), np.sin(self.init[2]), color='b', width=5)

        plt.show()

# obs = []
# obs.append((0,0,100,100))
# obs.append((2,2,1,1))
# target = (8, 8, math.pi)
# e = Environment(1000, 1000)
# e.setObs(obs)
# print(e.colDetect((100, 146, -math.pi/10)))
# print(e.stateColDetect((75, 45, 0), (75, 145, 0)))
# print(e.stateColDetect((75, 45, 0), (75, 135, -math.pi/10)))
# state, controlInputs = e.robot.computeMove(e.target, 0.3, 1.5, -0.3, 0.1)
# print('1')