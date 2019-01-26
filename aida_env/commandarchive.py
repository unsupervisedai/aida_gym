# -*- coding: utf-8 -*-
"""
Created on Wed Jan 16 14:50:08 2019

@author: nicolas & guillaume
"""
import numpy as np

### Gives multiple instances of command matrices for basic training
### 2 types of training (position or velocity) : choose commandtype = "position" or commandtype = "velocity" in aida_gym_env.py

### 1 - Position matrices : We give a set of (x,y) coordinates from which the robot has to deviate as little as possible
### We give here 8 cardinal straight lines with 200 coordinates each covering 20m of movement.

forward1 = [[x,0] for x in np.linspace(0,20,200)]

leftward1 = [[0,y] for y in np.linspace(0,20,200)]

rightward1 = [[0,-y] for y in np.linspace(0,20,200)]

backward1 = [[-x,0] for x in np.linspace(0,20,200)]

diagonal1 = [[x,x] for x in np.linspace(0,20,200)]

diagonal2 = [[x,-x] for x in np.linspace(0,20,200)]

diagonal3 = [[-x,x] for x in np.linspace(0,20,200)]

diagonal4 = [[-x,-x] for x in np.linspace(0,20,200)]

### We then give a few instances of more elaborate movement :

elbow1 = [[x,0] for x in np.linspace(0,10,100)] + [[0,y] for y in np.linspace(0,10,100)] ## forward then left

elbow2 = [[x,0] for x in np.linspace(0,10,100)] + [[x,x] for x in np.linspace(0,10,100)] + [[0,y] for y in np.linspace(0,10,100)] # A softer elbow

sqrt = [[x,(5*x)**.5] for x in np.linspace(0,20,200) ] # Movement without straight edges to the left (follows the sqrt graph)



### 2 - Velocity matrices
### The first commands will be a straight input of linear velocity for 10⁶ timesteps

xvel = [[10**6,[1,0,0,0,0,0]]]
yvel = [[10**6,[0,1,0,0,0,0]]]

xnegvel = [[10**6,[-1,0,0,0,0,0]]]
ynegvel = [[10**6,[0,-1,0,0,0,0]]]

###We then give more complex velocity commands :

stop = [[10**3,[1,0,0,0,0,0]] , [10**3,[0,0,0,0,0,0]], [10**3,[1,0,0,0,0,0]]] # Goes forward for 1000 steps, stops for 1000 and then goes towards x for 1000 steps again
turn = [[10**3,[1,0,0,0,0,0]] , [10**3,[0,1,0,0,0,0]]]
