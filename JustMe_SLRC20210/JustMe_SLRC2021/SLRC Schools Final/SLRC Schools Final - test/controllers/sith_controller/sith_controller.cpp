//Team name :JustMe

// TODO: Wall meassurements + line following accuracy

#include <webots/Robot.hpp>
//////// Other Libraries ////////////

#include <webots/DistanceSensor.hpp>
#include <Webots/Motor.hpp>

/////////////////////////////////////

/////// Constants ///////////////////

#define TIME_STEP 32

// Speed Variables
#define MAXSPEED 4.0
#define RIGHT 1
#define LEFT -1

// Threshold value
#define THRESH 400
#define DIST 180
// Turn deleays
#define SLIGHT_TURN_DELAY 2
#define pi_2 1.57
// Pillar sensor debounce delay
#define DEBOUNCE 8
#define SKIP 4

/////////////////////////////////////


using namespace webots;

// Class for the line following robot
class LineFollower
{
  private:  
   static const int MOTOR_NUM=4;
   static const int DS_NUM=8;
   static const int PS_NUM=2;
   
  public:
    
    DistanceSensor *ds[DS_NUM];
    DistanceSensor *dsFr;
    DistanceSensor *ps[PS_NUM];
    Robot *robot;
    Motor *motor[MOTOR_NUM];

    int sum=0;
    unsigned int binary=0;           // Variable to hold floor sensor readings
    unsigned int binaryCount=0;
    unsigned int frontVal=0;
    int data_ps[2] = {0,0};   // Array to hold pillar sensor readings
    
    // constructor
    LineFollower(Robot* robot)
    {
      this->robot = robot; 
      
      char dsNames[8][4] = {"IR1","IR2","IR3","IR4","IR5","IR6","IR7","IR8"};
      char dsFront[4] = "IR9";
      char psNames[2][4] = {"PSL","PSR"};
      char motorNames[4][18] = {"Motor_LF","Motor_LB","Motor_RF","Motor_RB"};    
      
      // Initialize distance sensors
      for(int i=0;i<this->DS_NUM;i++)
      {
        this->ds[i]=this->robot->getDistanceSensor(dsNames[i]);
        this->ds[i]->enable(TIME_STEP);
      }
      this->dsFr=this->robot->getDistanceSensor(dsFront);
      this->dsFr->enable(TIME_STEP);
      
      // Initialize distance sensors
      for(int i=0;i<this->PS_NUM;i++)
      {
        this->ps[i]=robot->getDistanceSensor(psNames[i]);
        this->ps[i]->enable(TIME_STEP);
      }
      
      // Initialize motors
      for(int i=0;i<this->MOTOR_NUM;i++)
      {
        this->motor[i]=this->robot->getMotor(motorNames[i]);
        this->motor[i]->setPosition(-INFINITY);
      }

    }
    
    // method to read line
    void read_line()
    {
      unsigned int bitmask=0b10000000;
      this->binary=0;
      
      for(int i=0;i<this->DS_NUM;i++){
        int val = this->ds[i]->getValue();
        (val<THRESH) ? this->binary |= bitmask : this->binary |= 0;
        bitmask = bitmask >> 1;
      }
      
      this->frontVal=0;
      int val = this->dsFr->getValue();
      if(val<THRESH) this->frontVal=1;
    }
       
    // method to read the number of sensors above line
    void count_binary(){
      unsigned int bitmask=0b10000000;
      this->binaryCount=0;
       for(int i=0;i<this->DS_NUM;i++){
        if(this->binary & bitmask)
            this->binaryCount++;
         bitmask = bitmask>>1;
       }
    }
    
    // method to follow the line
    void follow_line()
    {
	count_binary();
	switch(this->binaryCount)
	{
	    case 0 :          if(this->frontVal)
                    	               set_motors(0.8,0.8);
                    	           else
                      	               break;
	    
	    case 1 :	if(this->binary == 0b10000000)
  			    set_motors(-0.75, 1);
  			else if(this->binary == 0b00000001)
      			     set_motors(1, -0.75);
      			else if(this->binary == 0b00001000)
      			     set_motors(1, 0.15);
      			else if(this->binary == 0b00010000)
      			     set_motors(0.15, 1);
      			else if(this->binary == 0b00000100)
      			     set_motors(1, 0.25);
      			else if(this->binary == 0b00100000)
      			     set_motors(0.25, 1);
  			break;
					
	    case 2 :	if(this->binary == 0b00011000)
			    set_motors(1,1);						
			else if(this->binary == 0b11000000)
			    set_motors(-1, 1);
			else if(this->binary == 0b00000011)
			    set_motors(1, -1);
			else if(this->binary == 0b01100000)
			    set_motors(-0.95, 1);
			else if(this->binary == 0b00000110)
			   set_motors(1, -0.95);
			else if(this->binary == 0b00001100)
			   {if(this->frontVal)set_motors(0.8, 0.8);
			    else set_motors(1, -0.85);}
			else if(this->binary == 0b00110000)
			    {if(this->frontVal)set_motors(0.8, 0.8);
			     else set_motors(-0.85, 1);}
			break;
					
	    case 3 :	if(this->binary == 0b11100000)
			    set_motors(-1, 1);
			else if(this->binary == 0b00000111)
			    set_motors(1, -1);
			else if(this->binary == 0b01110000)
			    set_motors(-0.5, 1);
			else if(this->binary == 0b00001110)
			    set_motors(1, -0.5);
			else if(this->binary == 0b00111000)
			    set_motors(-0.35, 1);
			else if(this->binary == 0b00011100)
			    set_motors(1, -0.35);
			break;
	}
}
    
       
    // method to remain in the existing state
    void remain(int period)
    {
      // remain state
      int count=0;
      while (++count<period)
        this->robot->step(TIME_STEP);
    }
    
    // Methods for motion
    void set_motors(double leftSpeed, double rightSpeed){
        this->motor[0]->setVelocity(leftSpeed*MAXSPEED);
        this->motor[1]->setVelocity(leftSpeed*MAXSPEED);
        this->motor[2]->setVelocity(rightSpeed*MAXSPEED);
        this->motor[3]->setVelocity(rightSpeed*MAXSPEED);
    }
    
    // Methods for motion
    void forward(){
      for(int i=0;i<this->MOTOR_NUM;i++){
        this->motor[i]->setVelocity(MAXSPEED);
      }
    }

    // Method to turn
    void rotate_acute(int side){
    //Move forward until the middle sensors are out of the line   
      do{
          read_line();
          
          if((side==LEFT && (this->binary & 0b00000001)) || (side==RIGHT && (this->binary & 0b10000000)))
              return;
                
      }while(((this->binary & 0b00010000) || (this->binary & 0b00001000)) && robot->step(TIME_STEP)!=-1);
    // Rotate
    remain(5);
    if(side==RIGHT)
      set_motors(0.5, -0.5);
      
     else
       set_motors(-0.5, 0.5);
     get_nearest_pillar();
    // Rotate until either of the 2 middle sensors are on the line
    while((this->binary & 0b00010000 && this->binary & 0b00001000) && robot->step(TIME_STEP)!=-1)
        read_line(); 
    }
    
    
    void rotate(int side){
      set_motors(0.8, 0.8);
      rotate_acute(side);     
    }
    
    void t_rotate(int side){
      if(side==RIGHT)
        set_motors(0.5, -0.5);

       else
         set_motors(-0.5, 0.5);

       remain(5);
      // Rotate until either of the 2 middle sensors are on the line
      while(!(this->binary & 0b00010000 && this->binary & 0b00001000 && this->frontVal) && robot->step(TIME_STEP)!=-1)
          read_line(); 
    
    }
    
    void tf_rotate(int side){
     //Move forward until the middle sensors are out of the line   
      do{
          read_line();                
      }while(((this->binary & 0b00010000) || (this->binary & 0b00001000)) && robot->step(TIME_STEP)!=-1);
    // Rotate
    if(side==RIGHT)
      set_motors(0.5, -0.5);
     else
       set_motors(-0.5, 0.5);
    // Rotate until either of the 2 middle sensors are on the line
    while((this->binary & 0b00010000 && this->binary & 0b00001000) && robot->step(TIME_STEP)!=-1)
        read_line(); 
    }
    
    void stop(){
      for(int i=0;i<this->MOTOR_NUM;i++){
        this->motor[i]->setVelocity(0);
      }
      remain(10);
    }
    
    void get_nearest_pillar(){
      int values[2];
      for(int i=0;i<this->PS_NUM;i++)
       {
         int val = this->ps[i]->getValue();
         if(val<=260) values[i]=13;
         else if(val<=360) values[i]=18;
         else if(val<=460) values[i]=23;
         else values[i]=1000;
       }
      
      // Check if values are out of range
      if(values[0]>600 && values[1]>600) return;
           
      if(values[0]<values[1]){
        this->sum+=3*values[0];
        std::cout<<"Pillar at left side - "<<values[0]<<"cm"<<std::endl;
      }
      
      else{
        this->sum+=5*values[1];
        std::cout<<"Pillar at right side - "<<values[1]<<"cm"<<std::endl;
      }
    }
    
};

int main(int argc, char **argv) 
{
  /////////// Setup Code //////////////////
  bool start=true;       // Variable to keep track of start condition
  //bool stop=false;       // Variable to keep track of stop condition
  unsigned int box_count=0;
  int next_turn=0;
  
  Robot *robot = new Robot();
  
    /* Input Data Behavior:
      White --> value < 50.0
      Black --> value > 50.0
  */ 
  
  
  // initialize linefollowing robot
   LineFollower sith = LineFollower(robot);
  
  /////////////////////////////////////////
  
  /////////// Main Loop //////////////////
  
  while(robot->step(TIME_STEP)!=-1){
    // Logic for the motion of the robot
    while(start){
      sith.forward();
      sith.remain(30);
      start=false;
     }
    // Read data - IR sensors
    sith.read_line();
    sith.count_binary();
  

  switch(sith.binaryCount)
  {
      case 1 :  sith.follow_line();
                    break;
      case 2 :  sith.follow_line();
                    break;
      case 3 :  sith.follow_line();
                  if(sith.binary==0b00011001)
                      sith.rotate_acute(RIGHT);
                  else if(sith.binary==0b10011000)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b10110000)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b0001101)
                      sith.rotate_acute(RIGHT);
                  else if(sith.binary==0b10001100)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b00110001)
                      sith.rotate_acute(RIGHT);
                  break;
       
       case 4 :   if(sith.binary==0b11110000 || sith.binary==0b01111000)
                      {if(sith.frontVal) sith.t_rotate(LEFT);
                      else sith.rotate(LEFT);}
                  else if(sith.binary==0b00001111 || sith.binary==0b00011110)
                      {if(sith.frontVal) sith.t_rotate(RIGHT);
                      else sith.rotate(RIGHT);}
                  else if(sith.binary==0b00111100)
                      sith.rotate_acute(RIGHT);
                  else if(sith.binary==0b00110011)
                      sith.rotate_acute(RIGHT);
                  else if(sith.binary==0b11011000)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b11001100)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b00011011)
                      sith.rotate_acute(RIGHT);
                  else if(sith.binary==0b11101000)
                      sith.rotate(LEFT);
                  else if(sith.binary==0b00010111)
                      sith.rotate(RIGHT);
                  else if(sith.binary==0b10111000)
                      sith.rotate(LEFT);
                  else if(sith.binary==0b00011101)
                      sith.rotate(RIGHT);
                  else if(sith.binary==0b10011100)
                      sith.rotate_acute(LEFT);
                  else if(sith.binary==0b00111001)
                      sith.rotate_acute(RIGHT);
                  break;
                  
       case 5 :   if(sith.binary == 0b11111000 || sith.binary == 0b01111100)
		{if(sith.frontVal) sith.t_rotate(LEFT);
		else sith.rotate(LEFT);}
        	       else if(sith.binary == 0b00011111 || sith.binary == 0b00111110)
		{if(sith.frontVal) sith.t_rotate(RIGHT);
		else sith.rotate(RIGHT);}
    	       else if(sith.binary == 0b11101100)
		sith.rotate(LEFT);							
	       else if(sith.binary == 0b00110111)
		sith.rotate(RIGHT);						
	       else if(sith.binary == 0b11100110)
		sith.rotate(LEFT);							
	       else if(sith.binary == 0b01100111)
		sith.rotate(RIGHT);						
	       else if(sith.binary == 0b00111101)
		sith.rotate(RIGHT);						
	       else if(sith.binary == 0b10111100)
		sith.rotate(LEFT);							
	       else if(sith.binary == 0b00111011)
		sith.rotate(RIGHT);						
	       else if(sith.binary == 0b11011100)
		sith.rotate(LEFT);
	       break;
	      
        case 6 :  if(sith.binary==0b11111100)
                      sith.rotate(LEFT);
                  else if(sith.binary==0b00111111)
                      sith.rotate(RIGHT);
                  else if(sith.binary==0b01111110)
                      sith.rotate(LEFT);
                  break;
                  
        case 7 :  break;
        
        case 8 : if(sith.frontVal==0){
                   sith.tf_rotate(LEFT);
                 }
                 break;
  }
  
  
  }
  
  ////////////////////////////////////////
  
  /////////// Clean up code /////////////
  // print the pillars

  delete robot;
  
  ///////////////////////////////////////
  return 0;
}



