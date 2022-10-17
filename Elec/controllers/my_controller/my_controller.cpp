
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

int TIME_STEP = 64;


using namespace webots;


int main(int argc, char **argv) {
  
  Robot *robot = new Robot();

  Motor *left_motor = robot->getMotor("LeftMotor");
  Motor *right_motor = robot->getMotor("RightMotor");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  left_motor->setVelocity(10.0);
  right_motor->setVelocity(10.0);


  
  
  while (robot->step(TIME_STEP) != -1) {
    
  };


  delete robot;
  return 0;
}
