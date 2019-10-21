import env
import numpy as np
import math
import pygame
import time


e = env.Environment(8, 8)

clk_right = [2,3,4]
clk_left = [8,9,10]
clk_up = [11,0,1]
clk_down = [5,6,7]
clk_h = clk_right + clk_left
clk_v = clk_up + clk_down

# 3(a)
pi0 = dict()

for s in e.getStates():

    if s[0] == 5 and s[1] == 6 :
        m = env.Motion.none
    elif (s[0] <= 5 and s[2] in clk_right) or (s[0] >= 5 and s[2] in clk_left) or \
    (s[1] <= 6 and s[2] in clk_up) or (s[1] >= 6 and s[2] in clk_down) :
        m = env.Motion.forwards
    else:
        m = env.Motion.backwards
    
    if (s[0] == 5 and s[1] == 6) or (s[0] == 5 and s[2] in clk_v) or (s[1] == 6 and s[2] in clk_h): 
        r = env.Rotation.none
    elif (s[2] in clk_right and s[1] < 6) or (s[2] in clk_left and s[1] > 6) or \
    (s[2] in clk_up and s[0] > 5) or (s[2] in clk_down and s[0] < 5) :
        r = env.Rotation.left
    else:
        r = env.Rotation.right
    
    pi0[s] = (m, r) # initial policy

# 3(b)
def gen_traj(e, pi, s0, pe):
    s = s0
    traj = [s[0:2]]
    while s[0:2] != (5, 6):
        a = pi[s]
        s = e.step(pe, s, a)
        print(s)
        traj.append(s)
    plotTraj(traj)
    return traj

def plotTraj(traj):
    def drawArrow(start, end, color):
        pygame.draw.line(screen, (color, 0, 0), start, end, 5)
        rtt = math.degrees(math.atan2(start[1] - end[1], end[0] - start[0])) + 90
        pygame.draw.polygon(screen, (color, 0, 0), 
            ((end[0] + 15 * math.sin(math.radians(rtt)),
              end[1] + 15 * math.cos(math.radians(rtt))),
             (end[0] + 15 * math.sin(math.radians(rtt - 120)),
              end[1] + 15 * math.cos(math.radians(rtt - 120))),
             (end[0] + 15 * math.sin(math.radians(rtt + 120)),
              end[1] + 15 * math.cos(math.radians(rtt + 120)))))

    def adjustXY(xy):
        return (xy[0] * 100 + 50, 750 - xy[1] * 100)

    pygame.init()
    screen = pygame.display.set_mode([800, 800])
    
    done = False
    clock = pygame.time.Clock()
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        
        # draw grid
        screen.fill((255, 255, 255))
        for i in range(0, 801, 100):
            pygame.draw.line(screen, (0, 0, 0), (i, 0), (i, 800), 3)
            pygame.draw.line(screen, (0, 0, 0), (0, i), (800, i), 3)
        
        # plot trajectory
        colorDiff = abs(255/len(traj)) / 2
        for i in range(1, len(traj)):
            drawArrow(adjustXY(traj[i-1]), adjustXY(traj[i]), 100 + colorDiff * i)
        
        clock.tick(15)
        pygame.display.flip()

    pygame.quit()
    return

# 3(c)
#tr = gen_traj(e, pi0, (1, 6, 6), 0)
def expectation(e, s, a, V, pe, l):
    ex = 0
    for sp in e.getNextStateCandi(s, a):
        ex += e.transProb(pe, s, a, sp) * (e.reward(sp) + l * V[sp])
    return ex

# 3(d)
def policy_eval(e, pi, pe, l):
    V = np.zeros((e.L, e.W, 12))
    threshold = 1e-4
    while True:
        delta = 0
        for s in e.getStates():
            a = pi[s]
            v = expectation(e, s, a, V, pe, l)
            delta = max(delta, np.abs(v - V[s]))
            V[s] = v
        #print(delta)
        if delta < threshold:
            break
    return V

# 3(e)
#V = policy_eval(e, pi0, 0, 0.9)
#for stt in e.getStates():
#    print(stt, V[stt])

# 3(f)
def policy_refine(e, pe, l, V):
    pi = dict()
    for s in e.getStates():  
        max = -math.inf
        for a in e.getActions():
            if a[0] == env.Rotation.none and s[0:2] != (5, 6): continue
            ex = expectation(e, s, a, V, pe, l)
            #print(s, a, ex)
            if ex > max:
                max = ex
                pi[s] = a
        #print("*" * 50)
        #print(s, max, pi[s])
        #print("-" * 50)
    return pi

#policy_refine(e, 0, 0.9, V)
#print(policy_refine(e, 0, 0.9, V))
# 3(g)

def policyIteration(e, pe, l, pi0):
    pip = pi0
    pi = None
    i = 0
    while pip != pi:
        pi = pip
        print(i)
        i += 1
        V = policy_eval(e, pi, pe, l)
        pip = policy_refine(e, pe, l, V)
    return pip

# 3(h)
#pistar = policyIteration(e, 0, 0.9, pi0)
#gen_traj(e, pistar, (1, 6, 6), 0)

# 3(i)
class Timer(object):
    """docstring for Timer"""
    def __init__(self):
        super(Timer, self).__init__()
        self.startTime = 0

    def start(self):
        self.startTime = time.time()

    def end(self):
        t = time.time() - self.startTime
        print("Time Usage: {} sec".format(t))

timer = Timer()
timer.start()
pistar = policyIteration(e, 0.25, 0.9, pi0)
timer.end()
gen_traj(e, pistar, (1, 6, 6), 0.25)