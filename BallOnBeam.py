#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Physics Simulation of Ball on Beam
PID Controller to stabilized ball
Created on Tue Mar  3 03:12:50 2020

@author: bob walker
"""


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import sin, tan, pi
import random

# Ball on Beam class
class BallOnBeam:
    
    
    def __init__(self, loc0=0.0, vel0=0.0, theta=0.0, r=0.0127, dt=0.01475):
        self.loc0 = loc0
        self.vel0 = vel0
        self.theta = theta
        self.locs = []
        self.thetas =[]
        self.r = r
        self.dt = dt
        self.locs = []
        self.thetas = []
        self.accel = 0.0
        random.seed(7777)
        
        
    def reset(self, loc0=0.0, vel0=0.0, theta=0.0, r=0.0127, dt=0.01475):
        self.loc0 = loc0
        self.vel0 = vel0
        self.theta = theta
        self.locs = []
        self.thetas =[]
        self.r = r
        self.dt = dt
        self.locs = []
        self.thetas = []
        self.accel = 0.0
        random.seed(7777)
        
           
    def deg2rad(self, d):
        RAD2DEG = pi/180.0
        return d*RAD2DEG
         
    def step(self, theta):
        G = 9.81   # m/s^2
        wdot = 5.0*G*sin(self.deg2rad(theta))/self.r
        k = 1.0
        self.accel = k*wdot
        self.vel0 = self.vel0 + self.accel*self.dt
        self.loc0 = self.loc0 + self.vel0*self.dt
        
        # elastic collision
        if abs(self.loc0)>48.0:
            self.vel0 = -self.vel0
        
        self.vel0 *= 0.9995
        
        return self.loc0

    def elev(self, theta):  
        theta_Rad = self.deg2rad(theta)
        return -tan(theta_Rad)*self.loc0
    
    def beam(self, theta=0):
        global G, R, DT, RAD2DEG
        theta_Rad = self.deg2rad(theta)
        xbeam=[-100, 100]
        ybeam=[100*sin(theta_Rad), -100*sin(theta_Rad)]
        return xbeam, ybeam

 
bb = BallOnBeam(loc0=0.0, vel0=0.0, theta=0.0)
for i in range(30):
    x=bb.step(-1.0)
    y=bb.elev(x)
    print(x,y)
    

# plotting stuff
fig = plt.figure(figsize=(8,4))
ax = plt.axes(xlim=(-50, 50), ylim=(-10, 10), title='PID Controller to Stabilize Ball on Beam')
ax.plot([0,0],[0,0], color='orange', marker='o', markersize=2)
ax.grid()
ax.set_xlabel('Mar. 4, 2020  (Stabilized with PID Controller -- Elastic Wall Collision)')

line0, = ax.plot([], [], lw=1, color='magenta', marker='o', markersize=30) # ball
line1, = ax.plot([], [], lw=6, color='black', marker='o',markersize=1) # beam
line2, = ax.plot([], [], lw=5, color='orange', marker='o',markersize=5) # center of beam
line3, = ax.plot([], [], lw=2, color='gray', marker='o',markersize=5) # cbeam2
line4, = ax.plot([], [], lw=4, color='blue', marker='',markersize=8, alpha=0.5) # cbeam2
line5, = ax.plot([], [], lw=0, color='magenta', marker='*',markersize=30, alpha=0.15) # cbeam2

def init():
    print('2x')
    line0.set_data([], [])
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])

    
    return line0


xp=[]
yp=[]
theta = 0.0
targetTheta = 4.0
count=0
lastLoc0= 0.0
xTarget = 0.0
xSumErr = 0.0

def animate(i):
    global theta, xp, yp, count, targetTheta, lastLoc0, xTarget
    print('3')
    
    xp=[]
    yp=[]

    #theta =2.0
    x=bb.step(theta)
    y=bb.elev(theta)
    
    velStr='PID Controller -- Ball on Beam:  velocity={0:0.3f} (iteration={1:0d})'.format(bb.vel0, count)
    ax.set_title(velStr)

    xp.append(x)
    yp.append(y+1.5)

    #targetTheta = random.uniform(-2,2.00)
    
    if count<200:
        # unstable movement
        if x>0.01:
            targetTheta = -2.0
            
        if x<0.01:
            targetTheta = 2.0
    else:
        # very simple PID controller
        Kp=0.155
        Ki=0.0  #not used, yet
        Kd=5.25
        
        pid = -Kp*(x-xTarget) + Kd*(lastLoc0-x)
        print(pid)
        targetTheta=pid
        
        if abs(x-xTarget)<2.0:
            line0.set_color('green')
            line4.set_color('green')
            
            sx=[]
            sy=[]
            for istar in range(25):
                sx.append(random.uniform(-50.0, 50.0))
                sy.append(random.uniform(-10, 10))
            line5.set_data(sx,sy)
            
        else:
            line0.set_color('red')
            line4.set_color('red')
            line5.set_data([],[])
            
        # if bb.vel0>0.0:
        #     targetTheta= -4.0
        # else:
        #     targetTheta= 4.0
            
    
        
    if targetTheta>theta:
        theta += 0.195
        
    if targetTheta<theta:
        theta -= 0.195
     
    if count==599:
        xTarget=30
        
    if count>600:
        if count%200==0:
            xTarget=-xTarget
        
    if count>200:
        line4.set_data([xTarget-4, xTarget-4, xTarget+4, xTarget+4, xTarget-4],
                       [0.0, 3.2, 3.2, 0.0, 0.0])

        

    #theta=2.0
            
    line0.set_data(xp,yp) # ball
    
    # #xp=[]
    # #yp=[]

    # #line0.set_data([0],[0])
    
    xbeam, ybeam = bb.beam(theta)
    line1.set_data(xbeam, ybeam)  # beam

    line2.set_data([0],[0])  # center of beam
    line2.set_data([0],[0])
    
    line3.set_data(xbeam, ybeam) # fine line on beam
    

    
    count+=1
    lastLoc0 = x
    return line0, line1, line2, line3, line4, line5

# test code
# for i in range(10):
#     animate(1)
#animate(0)


anim = animation.FuncAnimation(fig, animate, init_func=init,
                              frames=1800, interval=30, blit=False, repeat=False)


anim.save('beamPID1.mp4', fps=35, extra_args=['-vcodec', 'libx264'])


plt.show()

# while True:
#     pass

    