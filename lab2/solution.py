import env
import numpy as np
import math

e = env.Environment(100, 100)
e.setTarget((80, 80, 0))
e.setInit((10, 10, math.pi))

t = 0
dt = 0.1
controlInputs = []
while t < 1 :
    controlInputs.append(e.robot.computeMove(e.target, 0.3, 1.5, -0.3, dt))
    t = t + dt



print('1')



