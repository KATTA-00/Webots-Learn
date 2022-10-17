
"""
#######  ROBO   ##########

chassy 
    lenght = 2x >>>>> 0.2
    width = x >>>>>>> 0.1
    hieght = x/5 >>>>> 0.04
    
    wheel = x/2 >>>>>> 0.05 + 0.01 + 0.001
    caster >>>> radii = 0.018

wheel 
    hieght = 0.02
    radii = 0.04

"""

# import modules
from controller import Robot
import numpy as np
import time

# create Robot obj
robot = Robot()

########## Define ############
#TIMESTEP
TIMESTEP = 64


def Drive(t):
    
    r = 0.02
    b = 0.05
    
    # equction
    a = 0.01
    p = 30
    q = 1/4
    
    x = a * t
    x_dot = a
    y_dot = p*q*np.cos(q*x) * x_dot
    teta_dot = p*q*(-np.sin(q*x) * x_dot) / ( 1 + p*np.cos(q*x)**2)
    teta = np.arctan(p*q*np.cos(q*x))
    
    
    A = r * np.cos(teta) / 2
    B = A
    C = r * np.sin(teta) / 2
    D = C
    E = r/(2*b)
    F = -E
    
    jacobian = np.array([[A,B],[C,D],[E,F]])
    In_jacobian = np.linalg.pinv(jacobian)
    
    M_R = In_jacobian[0][0]*x_dot + In_jacobian[0][1]*y_dot + In_jacobian[0][2]*teta_dot
    M_L = In_jacobian[1][0]*x_dot + In_jacobian[1][1]*y_dot + In_jacobian[1][2]*teta_dot
    
    return (M_L,M_R)


if __name__ == "__main__" :
    
    t = 1

    # get motors
    Right_Motor = robot.getDevice("MOTOR_R")
    Left_Motor = robot.getDevice("MOTOR_L")
    
    # set position
    Right_Motor.setPosition(float('inf'))
    Left_Motor.setPosition(float('inf'))
    
    # initial velosity to zero
    Left_Motor.setVelocity(0.0)
    Right_Motor.setVelocity(0.0)
    
    
    #Right_Motor.setVelocity(5.0)
    #Left_Motor.setVelocity(-5.0)
    
    
    # main loop
    while robot.step(TIMESTEP) != -1:
        SPEED = Drive(t)
        
        Left_Motor.setVelocity(SPEED[0])
        Right_Motor.setVelocity(SPEED[1])
        #time.sleep(1000)
        
        #time.sleep(3000)
        
        t+=1
        
        


