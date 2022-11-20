#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//Declarations
  // Program Constants
  //------Base------//
    #define DRIVER_INTERFACE 1     // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
    #define BASE_MAX_SPEED 2000
    #define BASE_RUN_SPEED 500
    #define BASE_ACCELERATION 300

  //------Shoulder------//
    #define SHOULDER_MAX_SPEED 2000
    #define SHOULDER_RUN_SPEED 500
    #define SHOULDER_ACCELERATION 200

  //------Elbow------//
    #define ELBOW_MAX_SPEED 12000
    #define ELBOW_RUN_SPEED 500
    #define ELBOW_ACCELERATION 1800

  //------Forarm------//
    #define FORARM_MAX_SPEED 2000
    #define FORARM_RUN_SPEED 500
    #define FORARM_ACCELERATION 200

  //------Wrist------//
    #define WRIST_MAX_SPEED 2000
    #define WRIST_RUN_SPEED 500
    #define WRIST_ACCELERATION 200

  // Base Pin Declarations
    #define BASE_LIMIT 10  //standard pin          
    #define BASE_ENA 11 //standard pin             
    #define BASE_PUL 12 //standard pin
    #define BASE_DIR 13 //PWM pin

  //Shoulder joint Declarations
    #define SHOULDER_LIMIT 52 //standard pin          
    #define SHOULDER_ENA 4  //standard pin (was pwm)           
    #define SHOULDER_PUL 5 //standard pin (was pwm)
    #define SHOULDER_DIR 6 //PWM pin

  //Elbow joint Declarations
    #define ELBOW_LIMIT 51 //standard pin          
    #define ELBOW_ENA 32 //standard pin           
    #define ELBOW_PUL 33 //standard pin 
    #define ELBOW_DIR 34 //PWM pin

  //Forarm joint Declarations
    #define FORARM_LIMIT 50
    #define FORARM_ENA 9
    #define FORARM_PUL 8
    #define FORARM_DIR 7

  //Wrist Joint Declarations
    #define WRIST_LIMIT 49 //standard pin          
    #define WRIST_ENA 47 //standard pin           
    #define WRIST_PUL 46 //standard pin 
    #define WRIST_DIR 45 //PWM pin

// Global Variables
  static long base_motor_position = 0;
  static long shoulder_motor_position = 0;
  static long elbow_motor_position = 0;
  static long forarm_motor_position = 0;
  static long wrist_motor_position = 0;


//static bool base_limit_reached = false;

// AccelStepper Object Instantiation
  AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
  AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
  AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
  AccelStepper ForarmStepper(DRIVER_INTERFACE, FORARM_PUL, FORARM_DIR);
  AccelStepper WristStepper(DRIVER_INTERFACE, WRIST_PUL, WRIST_DIR);

// ----- SETUP -----
void setup(){
  Serial.begin(9600);
  driverInit();
  //input_settings();  
  homeBaseAxis();
  //homeShoulderAxis();
  //homeElbowAxis();
  //homeWristAxis();
  

}

// ----- MAIN -----
void loop(){ 
   /*
   BaseStepper.enableOutputs();
   ShoulderStepper.enableOutputs();
   ElbowStepper.enableOutputs();
   ForarmStepper.enableOutputs();
   WristStepper.enableOutputs();
   BaseStepper.run();
   ShoulderStepper.run(); 
   ElbowStepper.run();
   WristStepper.run();
   */
}
void input_settings(void){
 
  /*------Base Inputs------*/
   ///*
   BaseStepper.enableOutputs(); 
   BaseStepper.moveTo(0); //90 degrees counter clockwise fron the center
   BaseStepper.setAcceleration(BASE_ACCELERATION);
   //*/
  /*------Shoulder Inputs------*/
  ///*
   ShoulderStepper.enableOutputs(); 
   ShoulderStepper.moveTo(0); //+ = forwards, - = backwards 2200 steps ~90 degrees
   //ShoulderStepper.moveTo(0);   
   ShoulderStepper.setAcceleration(SHOULDER_ACCELERATION);
   //*/
  /*------Elbow Inputs------*/
  ///*
   ElbowStepper.enableOutputs(); 
   ElbowStepper.moveTo(0); //+ = forwards, - = backwards 9000 ~90deg
   //ElbowStepper.moveTo(0);
   ElbowStepper.setAcceleration(ELBOW_ACCELERATION);
   //*/
  /*------Wrist Inputs------*/ 
  ///*
   WristStepper.enableOutputs(); 
   WristStepper.moveTo(0); //- = forwards, + = backwards 1750 steps ~90 degrees
   //WristStepper.moveTo(0);
   WristStepper.setAcceleration(WRIST_ACCELERATION); 
   //*/ 

}

//------Calibration Sequence for Base------//
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

//------Calibration Sequence for the Shoulder------//
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
    //ShoulderStepper.setAcceleration(300);
    //ShoulderStepper.runToNewPosition(-2200);//-2200 ~-90degrees
    ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //zero position is when the two arrows are lined up 
    shoulder_home_set = true;
    ShoulderStepper.disableOutputs();
  }
}
//------Calibration Sequence for the Elbow------//
void homeElbowAxis(void){
  bool elbow_home_set = false;
  int elbow_limit_reached = 0;

  ElbowStepper.enableOutputs();

  while(!elbow_home_set){

      while(!elbow_limit_reached){
        
      ElbowStepper.setSpeed(200); //Arm rotates forwards
      ElbowStepper.runSpeed();
      elbow_limit_reached = digitalRead(!ELBOW_LIMIT); //this pin stays low untill limit switch is hit 
    } 
    
    while(elbow_limit_reached){ //this runs when the limit switch is being held 
      ElbowStepper.setSpeed(-100);
      ElbowStepper.runSpeed();
      elbow_limit_reached = digitalRead(!ELBOW_LIMIT); 
    }
    
    ElbowStepper.setCurrentPosition(elbow_motor_position); //motor is set to zero and motor speed is zero
    ElbowStepper.setAcceleration(300);
    //ElbowStepper.runToNewPosition();//Zero point after calibration
    
    ElbowStepper.setCurrentPosition(elbow_motor_position);  //zero position is when the two arrows are lined up 
    elbow_home_set = true;
    ElbowStepper.disableOutputs();
  }

}

/*----- Forarm Axis Calibration------*/
void homeForarmAxis(void) {
  bool forarm_home_set = false;
  int forarm_limit_reached = 0;

  ForarmStepper.enableOutputs();

  while (!forarm_home_set) {

    while (!forarm_limit_reached) {  //motor rotates
      ForarmStepper.setSpeed(100);
      ForarmStepper.runSpeed();
      forarm_limit_reached = digitalRead(!FORARM_LIMIT);  //this pin stays low untill limit switch is hit
    }

    while (forarm_limit_reached) {  //this runs when the limit switch is being held
      ForarmStepper.setSpeed(-100);
      ForarmStepper.runSpeed();
      forarm_limit_reached = digitalRead(!FORARM_LIMIT);
    }

    ForarmStepper.setCurrentPosition(forarm_motor_position);  //motor is set to zero and motor speed is zero
    ForarmStepper.setAcceleration(300);
    //ForarmStepper.runToNewPosition();//needs to be tested
    ForarmStepper.setCurrentPosition(forarm_motor_position);  //zero position is when the two arrows are lined up
    ForarmStepper.disableOutputs();
    forarm_home_set = true;
  }
}


//------Calibration Sequence for the Wrist------//
void homeWristAxis(void){
  bool wrist_home_set = false;
  int wrist_limit_reached = 0;

  WristStepper.enableOutputs();

  while(!wrist_home_set){

      while(!wrist_limit_reached){
        
      WristStepper.setSpeed(200); //Arm rotates forwards
      WristStepper.runSpeed();
      wrist_limit_reached = digitalRead(!WRIST_LIMIT); //this pin stays low untill limit switch is hit 
    } 
    
    while(wrist_limit_reached){ //this runs when the limit switch is being held 
      WristStepper.setSpeed(-100);
      WristStepper.runSpeed();
      wrist_limit_reached = digitalRead(!WRIST_LIMIT); 
    }
    
    WristStepper.setCurrentPosition(wrist_motor_position); //motor is set to zero and motor speed is zero
    WristStepper.setAcceleration(300);
    //WristStepper.runToNewPosition();//Zero point after calibration
    
    WristStepper.setCurrentPosition(wrist_motor_position);  //zero position is when the two arrows are lined up 
    wrist_home_set = true;
    WristStepper.disableOutputs();
  }

}

//------AccelStepper Drivier Init ------//
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

  // Forarm
   pinMode(FORARM_LIMIT, INPUT);
   ForarmStepper.setMaxSpeed(FORARM_MAX_SPEED);
   ForarmStepper.setEnablePin(FORARM_ENA);
   ForarmStepper.setPinsInverted(false, false, true);
   ForarmStepper.disableOutputs();

  //Wrist
   pinMode(WRIST_LIMIT, INPUT);
   WristStepper.setMaxSpeed(WRIST_MAX_SPEED);
   WristStepper.setEnablePin(WRIST_ENA);
   WristStepper.setPinsInverted(false, false, true);
   WristStepper.disableOutputs(); 
   
}
