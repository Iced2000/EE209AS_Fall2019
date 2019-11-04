import env
import numpy as np
import math
import matplotlib.pyplot as plt

e = env.Environment(1000, 1000)
e.setTarget((800, 700, 0))
e.setInit((300, 200, math.pi/2))

# e.trajGen(e.init, e.target, 1, 0.05, 0.3, 1.5, -0.3, plot = True)        # k1,k2,k3 need to be modified

obs = [
    (0, 0, 100, 100),
    (500, 0, 200, 200),
    (700, 800, 200, 200),
    (0, 300, 300, 100),
    (0, 700, 200, 100),
    (800, 500, 200, 100),
    ]
e.setObs(obs)

# e.plotObs()
goalRegion = (500, 500, 50, 10)
e.setGoalRegion(goalRegion)
vertices, traj = e.rrt()

print('1')



