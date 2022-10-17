from controller import Robot

robot = Robot()

timestep = 64

lm = robot.getMotor("MOTOR_R")
rm = robot.getMotor("MOTOR_L")

lm.setPosition(float('inf'))
lm.setVelocity(0.0)

rm.setPosition(float('inf'))
rm.setVelocity(0.0)

sensors = []
names = ["LEFT_IR","FRONT_IR", "RIGHT_IR"]
reading = [0, 0, 0]

previous_error = 0.0
kp = 3  # 3
kd = 0.0  # 0.5
ki = 0.0
Integral = 0.0

for i in range(0, 3):
    sensors.append(robot.getDistanceSensor(names[i]))
    sensors[i].enable(timestep)


def getReading():
    for i in range(0, 3):
        if int(sensors[i].getValue()) > 512:
            reading[i] = 1
        else:
            reading[i] = 0
    # print(reading)


def PID():
    error = 0
    coefficient = [-1000,0, 1000]

    # [0,0,1,1,0,0,0,0]
    # error=coefficeint[0]*reading[0]+coeffficient[1]*reading[1]+________

    for i in range(0, 3):
        error += coefficient[i] * reading[i]

    P = kp * error
    I = Integral + (ki * error)
    D = kd * (error - previous_error)

    correction = (P + I + D) / 1000
    l_speed = 5 + correction
    r_speed = 5 - correction

    if l_speed < 0.0: l_speed = 0
    if l_speed > 10.0: l_speed = 10.0
    if r_speed < 0.0: r_speed = 0
    if r_speed > 10.0: r_speed = 10.0

    lm.setVelocity(l_speed)
    rm.setVelocity(r_speed)

    print(l_speed, r_speed, reading)
    return I, error


while (robot.step(timestep) != -1):
    getReading()

    print(kp, kd, ki)

    Integral, previous_error = PID()

# Enter here exit cleanup code.