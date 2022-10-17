from controller import Robot

robot = Robot()

TIMESTEP = 64

lm = robot.getDevice("MOTOR_R")
rm = robot.getDevice("MOTOR_L")

lm.setPosition(float('inf'))
lm.setVelocity(0.0)

rm.setPosition(float('inf'))
rm.setVelocity(0.0)

sensors = []
names = ["LEFT_IR","FRONT_IR", "RIGHT_IR"]
reading = [0, 0, 0]

previous_error = 0.0
kp = 3 # 3
kd = 0.5  # 0.5
ki = 0.00
Integral = 0.0

for i in range(0, 3):
    sensors.append(robot.getDevice(names[i]))
    sensors[i].enable(TIMESTEP)


def getReading():
    for i in range(0, 3):
        if int(sensors[i].getValue()) == 1000.0:
            reading[i] = 1
        else:
            reading[i] = 0
    # print(reading)


def PID():
    error = 0
    coefficient = [-2500,0, 2500]

    # [0,0,1,1,0,0,0,0]
    # error=coefficeint[0]*reading[0]+coeffficient[1]*reading[1]+________

    for i in range(0, 3):
        error += coefficient[i] * reading[i]

    P = error
    I = Integral + (error)
    D = (error - previous_error)

    correction = (kp* P + ki* I + kd*D) / 1000
    l_speed = 5 - correction
    r_speed = 5 + correction

    if l_speed < -2.5: l_speed = -2.5
    if l_speed > 9.0: l_speed = 9.0
    if r_speed < -2.5: r_speed = -2.5
    if r_speed > 9.0: r_speed = 9.0

    lm.setVelocity(l_speed)
    rm.setVelocity(r_speed)

    #print(l_speed, r_speed, reading)
    return I, error


while robot.step(TIMESTEP) != -1:
    getReading()
    print(reading)

    #print(kp, kd, ki)

    Integral, previous_error = PID()
    
    #print(sensors[2].getValue())

# Enter here exit cleanup code.
