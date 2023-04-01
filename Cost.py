import matplotlib.pyplot as plt
import numpy as np
import math

def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D
    

actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5]]
        
for action in actions:
     k=cost(0,0,45, action[0],action[1])      # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs
     print(k[3])
  
 

    
