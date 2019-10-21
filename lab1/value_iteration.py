import env 
import numpy as np
 
e = env.Environment(8, 8)

# 4(a)
def valueIteration(e, pe, l):
    pi = dict()
    V = np.zeros((e.L, e.W, 12))
    threshold = 1e-4
    while True:
        delta = 0
        for s in e.getStates():
            vmax = -np.Inf
            for a in e.getActions():
                v = env.expectation(e, s, a, V, pe, l)
                if v > vmax :
                    vmax = v 
                    pi[s] = a
            delta = max(delta, np.abs(V[s] - vmax))
            V[s] = vmax
        print(delta)
        if delta < threshold:
            break
    return  pi, V

# 4(b)
pistar, V = valueIteration(e, 0, 0.9)
tr = env.gen_traj(e, pistar, (1, 6, 6), 0)

# 4(c)
timer = env.Timer()
timer.start()
pistar, V = valueIteration(e, 0, 0.9)
timer.end()
