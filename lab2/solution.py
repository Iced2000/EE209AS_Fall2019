import env
import numpy as np
import math
import matplotlib.pyplot as plt

e = env.Environment(1000, 1000)
e.setTarget((800, 700, 0))
e.setInit((100, 500, math.pi/2))

# e.trajGen(e.init, e.target, 1, 0.05, 0.3, 1.5, -0.3, plot = True)        # k1,k2,k3 need to be modified

obs = [
    (200, 200, 100, 600),
    (500, 200, 100, 600),
    (800, 100, 100, 300),
    (800, 600, 100, 300),
    ]
e.setObs(obs)

# e.plotObs()
goalRegion = (900, 500, 30, 30)
e.setGoalRegion(goalRegion)
_, x = e.rrt()
e.plotTree(x)
e.tree = env.Tree([])
_, x = e.rrt_star()
e.plotTree(x)

print('1')



