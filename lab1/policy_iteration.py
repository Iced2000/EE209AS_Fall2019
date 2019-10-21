import env
import numpy as np

# set up the environment
e = env.Environment(8, 8)

# define some sets of clk status
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
    
    pi0[s] = (m, r)

# 3(b)
# find gen_traj() in env.py

# 3(c)
tr = env.gen_traj(e, pi0, (1, 6, 6), 0)

# 3(d)
def policy_eval(e, pi, pe, l):
    V = np.zeros((e.L, e.W, 12))
    threshold = 1e-4
    while True:
        delta = 0
        for s in e.getStates():
            a = pi[s]
            v = env.expectation(e, s, a, V, pe, l)
            delta = max(delta, np.abs(v - V[s]))
            V[s] = v
        #print(delta)
        if delta < threshold:
            break
    return V

# 3(e)
V = policy_eval(e, pi0, 0, 0.9)
print(V[(1, 6, 6)])

# 3(f)
def policy_refine(e, pe, l, V):
    pi = dict()
    for s in e.getStates():  
        exmax = -np.Inf
        for a in e.getActions():
            if a[0] == env.Motion.none and s[0:2] != (5, 6): continue
            ex = env.expectation(e, s, a, V, pe, l)
            if ex > exmax:
                exmax = ex
                pi[s] = a
    return pi

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
pistar = policyIteration(e, 0, 0.9, pi0)
env.gen_traj(e, pistar, (1, 6, 6), 0)

# 3(i)
timer = env.Timer()
timer.start()
pistar = policyIteration(e, 0, 0.9, pi0)
timer.end()

# 5(a)
pistar = policyIteration(e, 0.25, 0.9, pi0)
env.gen_traj(e, pistar, (1, 6, 6), 0.25)
