"""my_controller_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time 

TIMESTEP = 64

# create the Robot instance.
robot = Robot()



if __name__ == "__main__":
    t1 = time.time()
    
    motor1 = robot.getDevice("MOTOR_1")
    #motor2 = robot.getDevice("MOTOR_2")
    #M1 = robot.getDevice("M1")
    
    motor1.setPosition(float('inf'))
    #motor2.setPosition(float('inf'))
    #M1.setPosition(float('inf'))
    
    motor1.setVelocity(5.0)
    #motor2.setVelocity(0.0)
    #M1.setVelocity(1.0)
    
    #motor2.setVelocity(1.0)
    
    
    
    while robot.step(TIMESTEP) != -1:
        
        pass
       
        
        
