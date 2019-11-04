import env
import numpy as np
import math
import matplotlib.pyplot as plt

e = env.Environment(100, 100)
e.setTarget((80, 80, 0))
e.setInit((10, 10, math.pi/2))

# e.trajGen(e.init, e.target, 1, 0.05, 0.3, 1.5, -0.3)        # k1,k2,k3 need to be modified

obs = [
    (0, 0, 10, 10),
    (50, 0, 20, 20),
    (70, 80, 20, 20),
    (0, 30, 30, 10),
    (0, 70, 20, 10),
    (80, 50, 20, 10),
    ]
e.setObs(obs)

# e.plotObs()
goalRegion = [(50, 50, 10, 10)]





print('1')



