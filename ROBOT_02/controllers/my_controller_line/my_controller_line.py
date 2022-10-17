
from controller import Robot
import time
 
TIMESTEP = 64
robot = Robot()


def Stop(r,l):
    r.setVelocity(0.0)
    l.setVelocity(0.0)

def Go_straight(r,l):
    r.setVelocity(5.0)
    l.setVelocity(5.0)

def Turn_right(r,l):
    r.setVelocity(-3.0)
    l.setVelocity(3.0)
    
def Turn_right_bit(r,l,x):
    if x == 0:
        r.setVelocity(0.0)
        l.setVelocity(7.0)
    elif x == 1:
        r.setVelocity(-1.0)
        l.setVelocity(6.0)

def Turn_left(r,l):
    r.setVelocity(3.0)
    l.setVelocity(-3.0)

def Turn_left_bit(r,l,x):
    if x == 0:
        r.setVelocity(7.0)
        l.setVelocity(0.0)
    elif x == 1:
        r.setVelocity(6.0)
        l.setVelocity(-1.0)   


if __name__ == "__main__" :
    right_motor = robot.getDevice("MOTOR_R")
    left_motor = robot.getDevice("MOTOR_L")
        
    right_motor.setPosition(float('inf'))
    left_motor.setPosition(float('inf'))
    
    right_motor.setVelocity(0.0)
    left_motor.setVelocity(0.0)
    
    front_sen = robot.getDevice("FRONT_SEN")
    front_sen.enable(TIMESTEP)
    
    ir_front = robot.getDevice("FRONT_IR")
    ir_front.enable(TIMESTEP)
    ir_left = robot.getDevice("LEFT_IR")
    ir_left.enable(TIMESTEP)
    ir_right = robot.getDevice("RIGHT_IR")
    ir_right.enable(TIMESTEP)
        
    
    while robot.step(TIMESTEP) != -1:
            
            
            ir_front_val = ir_front.getValue()
            ir_left_val = ir_left.getValue()
            ir_right_val = ir_right.getValue()
            
            VAL = [ir_left_val == 1000.0, ir_front_val == 1000.0 , ir_right_val == 1000.0]
            
            #print(VAL)
            if VAL[2]:
                if VAL[1]:
                    Turn_right_bit(right_motor,left_motor,1)
                    print("right_bit_1")
                elif VAL[0] :
                    Turn_right_bit(right_motor,left_motor,0)
                    print("right_bit_0")
                    Stop(right_motor,left_motor)
                    break
                else:
                    Turn_right(right_motor,left_motor)
                
            elif VAL[0]:
                if VAL[1]:
                    Turn_left_bit(right_motor,left_motor,1)
                    print("left_bit_1")
                elif VAL[2] :
                    Turn_right_bit(right_motor,left_motor,0)
                    print("left_bit_0")
                else:
                    Turn_left(right_motor,left_motor)
               
            else:
                Go_straight(right_motor,left_motor)
            
        

# Enter here exit cleanup code.
