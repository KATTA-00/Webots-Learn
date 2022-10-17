from controller import Robot

robot = Robot()

TIMESTEP = 64

lm = robot.getDevice("RightMotor")
rm = robot.getDevice("LeftMotor")

lm.setPosition(float('inf'))
lm.setVelocity(0.0)

rm.setPosition(float('inf'))
rm.setVelocity(0.0)



rm.setVelocity(10.0)
lm.setVelocity(10.0)


while robot.step(TIMESTEP) != -1:
    pass
    


