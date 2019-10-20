import numpy as np
from enum import Enum

class Motion(Enum):
    none = 0
    forwards = 1
    backwards = 2

class Rotation(Enum):
    none = 0
    left = 1
    right = 2

class Environment(object):
    """docstring for Environment"""
    def __init__(self, L, W):
        super(Environment, self).__init__()
        self.L = L
        self.W = W

        # initiaizing state
        self.state = []
        for i in range(L):
            for j in range(W):
                for k in range(12):
                    self.state.append((i, j, k))

        # initializing action
        self.action = [(Motion.none, Rotation.none)]
        for m in Motion:
            for r in Rotation:
                if m == Motion.none: break
                self.action.append((m, r))

        self.stateLen = len(self.state)
        self.actionLen = len(self.action)

        # given s and a, the candidate of next state and prob
        self.nextStateCandi = dict()

    # 1(a)
    def getStates(self):
        return self.state

    # 1(b)
    def getActions(self):
        return self.action

    def getStateLen(self):
        return self.stateLen

    def getActionLen(self):
        return self.actionLen

    def getNextStateCandi(self, s, a):
        def clock2xy(clk, mot):
            xy = np.zeros(2).astype(int)
            direction = 1 if mot == Motion.forwards else -1
            if 2 <= clk <= 4:
                xy[0] = 1
            elif 5 <= clk <= 7:
                xy[1] = -1
            elif 8 <= clk <= 10:
                xy[0] = -1
            else:
                xy[1] = 1
            return xy * direction

        def rotate(clk, rot):
            if rot == Rotation.left:
                clk += 11
            elif rot == Rotation.right:
                clk += 1
            clk %= 12
            return clk

        def act(xy, preClk, a):
            xy = np.clip(xy + clock2xy(preClk, a[0]), 0, 7)
            postClk = rotate(preClk, a[1])
            return (*list(xy), postClk)

        try:
            return self.nextStateCandi[(s, a)][0]
        except:
            # stand still
            if a[0] == Motion.none:
                self.nextStateCandi[(s, a)] = [[s], [1]]
                return [s]
            # pre-rotation
            else:
                xy = np.array(s[0:2])
                nsc = []
                for r in Rotation:
                    nsc.append(act(xy, rotate(s[2], r), a))
                # True means normal, False means pre-rotation error
                self.nextStateCandi[(s, a)] = [nsc, [True, False, False]]
                return nsc

    # 1(c)
    def transProb(self, pe, s, a, sp):
        assert(pe <= 0.5 and s in self.getStates())
        assert(a in self.getActions() and sp in self.getStates())

        spCandi = self.getNextStateCandi(s, a)

        # stand still
        if a[0] == Motion.none:
            if sp in spCandi:
                return 1
            else:
                return 0

        # pre-rotation
        if sp in spCandi:
            if self.nextStateCandi[(s, a)][1][spCandi.index(sp)]:
                return 1 - 2 * pe
            else:
                return pe
        else:
            return 0

    # 1(d)
    def step(self, pe, s, a):
        prob = []
        for state in self.state:
            prob.append(self.transProb(pe, s, a, state))
        stateIdx = np.random.choice(range(self.getStateLen()), 1, p=prob)
        return self.getStates()[stateIdx[0]]

    # 2(a)
    def reward(self, s):
        assert(s in self.getStates())

        if s[0]%7 == 0 or s[1]%7 == 0:
            return -100
        elif s[0] == 3 and 6 >= s[1] >= 4:
            return -10
        elif s[0] == 5 and s[1] == 6:
            return 1
        else:
            return 0

env = Environment(8, 8)

print(env.step(0.5, (1, 1, 1), (Motion.forwards, Rotation.right)))
