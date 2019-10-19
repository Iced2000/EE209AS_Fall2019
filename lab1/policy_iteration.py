import env
import numpy as np
import math
import matplotlib.pyplot as plt
import turtle

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
    
    pi0[s] = (m, r)

# 3(b)
def gen_traj(e, pi, s0, pe):
    pen = turtle.Pen()
    pen.speed(10)
    width = 80
    count = 8
    o = width * count / 2

    for i in range(count + 1):
        pen.penup()
        pen.goto(-o, o - i * width)
        pen.pendown()
        pen.goto(o, o - i * width)

    for i in range(count + 1):
        pen.penup()
        pen.goto(-o + i * width, o)
        pen.pendown()
        pen.goto(-o + i * width, -o)
    
    pen.speed(1)
    pen.pencolor('red')

    s = s0
    traj_x = [s[0]]
    traj_y = [s[1]]
    while s[0:2] != (5, 6):
        a = pi[s]
        s = e.step(pe, s, a) 
        traj_x.append(s[0])
        traj_y.append(s[1])
        pen.penup()
        pen.goto(width/2 - o + traj_x[-2] * width, width/2 - o + traj_y[-2] * width)
        pen.pendown()
        pen.goto(width/2 - o + traj_x[-1] * width, width/2 - o + traj_y[-1] * width)
    # need some advice on how to plot this thing !!!

    turtle.done()

    return traj_x, traj_y


# 3(c)
traj_x, traj_y = gen_traj(e, pi0, (1, 6, 6), 0)


def expectation(e, s, a, V, pe, l):
    ex = 0
    for sp in e.getStates():
        ex += e.transProb(pe, s, a, sp) * (e.reward(sp) + l * V[sp])
    return ex

# 3(d)
def policy_eval(e, pi, pe, l):

    V1 = np.zeros((e.L, e.W, 12))
    V2 = np.zeros((e.L, e.W, 12))
    threshold = 0.01
    converge = False
    while ~converge:
        for s in e.getStates():
            print(s)
            a = pi[s]
            # V2[idx] = V1[idx] + e.reward(e.step(pe, s, a))
            V2[s] = expectation(e, s, a, V1, pe, l)
        converge = (abs(V1-V2) < threshold).all()
        V1 = V2
    return V2

# 3(e)
V = policy_eval(e, pi0, 0, 0.9) 
print(V[(1, 6, 6)])

# 3(f)
def policy_refine(e, pe, l, V):

    pi = dict()
    for s in e.getStates():  
        max = -math.inf
        for a in e.getActions():
            ex = expectation(e, s, a, V, pe, l)
            if ex > max:
                max = ex
                pi[s] = a
    return pi
            
# 3(g)







print('1')

        