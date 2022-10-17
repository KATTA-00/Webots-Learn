"""my_controller_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time 

import numpy as np

print(np.array([[1,2,4],[4,5,6]]))


TIMESTEP = 64

# create the Robot instance.
robot = Robot()

def driver(arr):
    if arr[1]:
        if arr[0] and not arr[2]:
            return "turn_right"
        elif arr[2] and not arr[0]:
            return "turn_left"
        elif not arr[0] and not arr[2]:
            return "turn_right"
        else:
            return "turn_back"
            
    elif arr[0]:
        if arr[0] and not arr[2]:
            return "turn_right"
        elif arr[2] and not arr[0]:
            return "turn_left"
        elif not arr[0] and not arr[2]:
            return "turn_right"
        else:
            return "turn_back"
            
    elif arr[2]:
        if arr[0] and not arr[2]:
            return "turn_right"
        elif arr[2] and not arr[0]:
            return "turn_left"
        elif not arr[0] and not arr[2]:
            return "turn_right"
        else:
            return "turn_back"
               
    else:
        return "go_straight"
    
    
        
    

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

if __name__ == "__main__":

    right_motor = robot.getDevice("MOTOR_L")
    left_motor = robot.getDevice("MOTOR_R")
    
    right_motor.setPosition(float('inf'))
    left_motor.setPosition(float('inf'))
    
    right_motor.setVelocity(0.0)
    left_motor.setVelocity(0.0)
    
    front_sen = robot.getDevice("front_sensor")
    front_sen.enable(TIMESTEP)
    right_sen = robot.getDevice("right_sensor")
    right_sen.enable(TIMESTEP)
    left_sen = robot.getDevice("left_sensor")
    left_sen.enable(TIMESTEP)
    
    right_motor_speed = 5.0
    left_motor_speed = 5.0
    
    while robot.step(TIMESTEP) != -1:
        
        val_front_sen = front_sen.getValue()
        val_right_sen = right_sen.getValue()
        val_left_sen  = left_sen.getValue()
        
        val = [val_left_sen < 600,val_front_sen < 600,val_right_sen < 600]
        print(val)
        
        go = driver(val)
        print(go)
        
        if go == "go_straight":
            right_motor_speed = 5.0
            left_motor_speed = 5.0
        if go == "turn_right":
            right_motor_speed = -2.0
            left_motor_speed = 2.0
        if go == "turn_left":
            right_motor_speed = 2.0
            left_motor_speed = -2.0
            
       
        right_motor.setVelocity(-right_motor_speed)
        left_motor.setVelocity(-left_motor_speed)
        



