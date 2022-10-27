#include <AccelStepper.h>
#include <MultiStepper.h>

// Program Constants
#define DRIVER_INTERFACE 1     // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
#define BASE_MAX_SPEED 2000
#define BASE_RUN_SPEED 500
#define BASE_ACCELERATION 300

#define SHOULDER_MAX_SPEED 2000
#define SHOULDER_RUN_SPEED 500
#define SHOULDER_ACCELERATION 200

#define ELBOW_MAX_SPEED 2000
#define ELBOW_RUN_SPEED 500
#define ELBOW_ACCELERATION 200

// Base Pin Declarations
#define BASE_LIMIT 3  //standard pin          
#define BASE_ENA 4  //standard pin             
#define BASE_PUL 5  //standard pin
#define BASE_DIR 6  //PWM pin

 //Shoulder joint Declarations
#define SHOULDER_LIMIT 10 //standard pin          
#define SHOULDER_ENA 9  //standard pin (was pwm)           
#define SHOULDER_PUL 8  //standard pin (was pwm)
#define SHOULDER_DIR 7  //PWM pin

 //Elbow joint Declarations
#define ELBOW_LIMIT 2 //standard pin          
#define ELBOW_ENA 11  //standard pin           
#define ELBOW_PUL 12 //standard pin 
#define ELBOW_DIR 13 //PWM pin

// Global Variables
static long base_motor_position = 0;
static long shoulder_motor_position = 0;
static long elbow_motor_position = 0;

// Interrupt Logic variables
//static bool base_limit_reached = false;

// AccelStepper Object Instantiation
AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);

// ----- SETUP -----
void setup(){
  Serial.begin(9600);
  driverInit();
  // attachInterrupt(digitalPinToInterrupt(BASE_LIMIT),isrBaseLimit, LOW); 
  //homeBaseAxis();
  //homeShoulderAxis();
  //homeElbowAxis();

}

// ----- MAIN -----
void loop(){ 
 /*------Base Inputs------*/
   /*
   BaseStepper.enableOutputs();  
   BaseStepper.moveTo(3000); //3000 steps~ 90 degree rotation
   BaseStepper.setAcceleration(BASE_ACCELERATION);
   BaseStepper.run();
   */ 
 /*------Shoulder Inputs------*/  
   ShoulderStepper.enableOutputs(); 
   ShoulderStepper.moveTo(1100); //+ = forwards, - = backwards. +- 2200 steps ~90 degrees
   ShoulderStepper.setAcceleration(SHOULDER_ACCELERATION);
   ShoulderStepper.run(); 
 /*------Elbow Inputs------*/ 
   /*
   ElbowStepper.enableOutputs(); 
   ElbowStepper.moveTo(800); //+ = forwards, - = backwards
   ElbowStepper.setAcceleration(ELBOW_ACCELERATION);
   ElbowStepper.run();
   */

}

// ----- Home Axis Algorithm------
void homeBaseAxis(void){  
  bool base_home_set = false;
  int base_limit_reached = 0;

  BaseStepper.enableOutputs();

  while(!base_home_set){

      while(!base_limit_reached){//motor rotates CCW 
      BaseStepper.setSpeed(500);
      BaseStepper.runSpeed();
      base_limit_reached = digitalRead(BASE_LIMIT); //this pin stays low untill limit switch is hit 
    } 
    
    while(base_limit_reached){ //this runs when the limit switch is being held 
      BaseStepper.setSpeed(-100);
      BaseStepper.runSpeed();
      base_limit_reached = digitalRead(BASE_LIMIT); 
    }
    
    BaseStepper.setCurrentPosition(base_motor_position); //motor is set to zero and motor speed is zero
    BaseStepper.setAcceleration(300);
    BaseStepper.runToNewPosition(-6000);//-6000 steps from the limit switch gets us very close to the zero point on the arm
    //~44.44 steps per degree
    BaseStepper.setCurrentPosition(base_motor_position);  //zero position is when the two arrows are lined up 
    base_home_set = true;
    BaseStepper.disableOutputs();
  }
}

/*
void homeShoulderAxis(void){  
  bool shoulder_home_set = false;
  int shoulder_limit_reached = 0;

  ShoulderStepper.enableOutputs();

  while(!shoulder_home_set){

      while(!shoulder_limit_reached){
        
      ShoulderStepper.setSpeed(200); //Arm rotates forwards
      ShoulderStepper.runSpeed();
      shoulder_limit_reached = digitalRead(SHOULDER_LIMIT); //this pin stays low untill limit switch is hit 
    } 
    
    while(shoulder_limit_reached){ //this runs when the limit switch is being held 
      ShoulderStepper.setSpeed(-100);
      ShoulderStepper.runSpeed();
      shoulder_limit_reached = digitalRead(SHOULDER_LIMIT); 
    }
    
    ShoulderStepper.setCurrentPosition(shoulder_motor_position); //motor is set to zero and motor speed is zero
    ShoulderStepper.setAcceleration(300);
    //ShoulderStepper.runToNewPosition();//
    
    ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //zero position is when the two arrows are lined up 
    shoulder_home_set = true;
    ShoulderStepper.disableOutputs();
  }
}
*/

// ----- AccelStepper Drivier Init -----
void driverInit(void){

  // Base 
  pinMode(BASE_LIMIT, INPUT);
   BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
   BaseStepper.setEnablePin(BASE_ENA);
   BaseStepper.setPinsInverted(false, false, true);
   BaseStepper.disableOutputs(); 

  // Shoulder
   pinMode(SHOULDER_LIMIT, INPUT);
   ShoulderStepper.setMaxSpeed(SHOULDER_MAX_SPEED);
   ShoulderStepper.setEnablePin(SHOULDER_ENA);
   ShoulderStepper.setPinsInverted(false, false, true);
   ShoulderStepper.disableOutputs(); 

  // Elbow
   pinMode(ELBOW_LIMIT, INPUT);
   ElbowStepper.setMaxSpeed(ELBOW_MAX_SPEED);
   ElbowStepper.setEnablePin(ELBOW_ENA);
   ElbowStepper.setPinsInverted(false, false, true);
   ElbowStepper.disableOutputs(); 
   
}

// ----- ISR's -----
void isrBaseLimit(void){
  //base_limit_reached = true;
}
