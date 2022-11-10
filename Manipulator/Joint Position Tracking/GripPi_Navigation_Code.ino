#include <Servo.h>

#include <AccelStepper.h>
#include <MultiStepper.h>
//------Declarations------// 
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

  #define WRIST_MAX_SPEED 2000
  #define WRIST_RUN_SPEED 500
  #define WRIST_ACCELERATION 200

  // Base Pin Declarations
  // Driver #1
    #define BASE_LIMIT 53  //standard pin  GREEN   
    #define BASE_ENA 11  //standard pin  WHITE             
    #define BASE_PUL 12  //standard pin  ORANGE
    #define BASE_DIR 13  //PWM pin YELLOW

 //shoulder joint 
 // Driver #3/#2
    #define SHOULDER_LIMIT 52 //standard pin  GREEN     (pull down circuit)     
    #define SHOULDER_ENA 4  //standard pin (was pwm)  WHITE         
    #define SHOULDER_PUL 5  //standard pin (was pwm)  ORANGE
    #define SHOULDER_DIR 6  //PWM pin YELLOW

 //Elbow joint Declarations
 // Driver #4
    #define ELBOW_LIMIT 51 //standard pin          
    #define ELBOW_ENA 32  //standard pin           
    #define ELBOW_PUL 33 //standard pin 
    #define ELBOW_DIR 34 //PWM pin

 //Wrist Joint Declarations
 // Driver #6 
    #define WRIST_LIMIT 50 //standard pin          
    #define WRIST_ENA 47 //standard pin           
    #define WRIST_PUL 46 //standard pin 
    #define WRIST_DIR 45 //PWM pin

//

//------Global Variables------//
  static long base_motor_position = 0;
  static long shoulder_motor_position = 0;
  static long elbow_motor_position = 0;
  static long wrist_motor_position = 0;
  int base_steps = 0;
  int shoulder_steps = 0;
  int elbow_steps = 0;
  int wrist_steps = 0;
  bool base_location_reached = 0;
  bool shoulder_location_reached = 0;
  bool elbow_location_reached = 0;
  bool wrist_location_reached = 0;
  static int PI_ARRAY_MATRIX[6]={0,0,0,0,0,0};
//

//------AccelStepper Object Instantiation------//
  AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
  AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
  AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
  AccelStepper WristStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
//

//------SETUP------//
void setup(){
  Serial.begin(9600);
  driverInit();
  //homeBaseAxis();
  //homeShoulderAxis();
  //homeElbowAxis();
  //homeWristAxis();
}
//

//------Main Loop------//
void loop(){ 
  //------while loop for when the location has already been reached------//
  while (base_location_reached==0 && shoulder_location_reached==0 && elbow_location_reached==0 & wrist_location_reached==0)
    {
      //outputs are disabled until a new posision is commanded
        BaseStepper.disableOutputs();
        ShoulderStepper.disableOutputs();
        ElbowStepper.disableOutputs();
        WristStepper.disableOutputs();
      
      /*-----------base inputs----------*/
        BaseStepper.moveTo(base_steps); //the pi will update this value base_steps 
        BaseStepper.setAcceleration(BASE_ACCELERATION); 
        BaseStepper.run(); 
        base_location_reached = BaseStepper.distanceToGo();

      /*---------shoulder inputs-------*/
        ShoulderStepper.moveTo(shoulder_steps); //the pi will update this value shoulder_steps
        ShoulderStepper.setAcceleration(SHOULDER_ACCELERATION);
        ShoulderStepper.run();  
        shoulder_location_reached=ShoulderStepper.distanceToGo();

      //------Elbow Inputs------//
        ElbowStepper.moveTo(elbow_steps); //the pi will update this value elbow_steps 
        ElbowStepper.setAcceleration(ELBOW_ACCELERATION);
        ElbowStepper.run();   
        elbow_location_reached=ElbowStepper.distanceToGo();

      //------Wrist Inputs------//  
        WristStepper.moveTo(wrist_steps); //the pi will update this value wrist_steps 
        WristStepper.setAcceleration(WRIST_ACCELERATION);
        WristStepper.run();  
        wrist_location_reached=WristStepper.distanceToGo();
    }

  //-------While loop to run while joints are moving------//
  while(base_location_reached != 0 || shoulder_location_reached != 0 || elbow_location_reached != 0 ||wrist_location_reached !=0)
    {
      //once the inputs are recieved, this loop plays traveling to the location
      BaseStepper.enableOutputs();
      ShoulderStepper.enableOutputs();
      ElbowStepper.enableOutputs();
      WristStepper.enableOutputs();
      base_location_reached = BaseStepper.distanceToGo();
      BaseStepper.run();
      shoulder_location_reached=ShoulderStepper.distanceToGo();
      ShoulderStepper.run();    
      elbow_location_reached = ElbowStepper.distanceToGo();
      ElbowStepper.run();
      wrist_location_reached = WristStepper.distanceToGo();
      WristStepper.run();
    }
}

//------Calibrations------//
  /*----- Base Axis Calibration------*/
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

  /*----- Shoulder Axis Calibration------*/
  void homeShoulderAxis(void){  
    bool shoulder_home_set = false;
    int shoulder_limit_reached = 0;

    ShoulderStepper.enableOutputs();

    while(!shoulder_home_set){

        while(!shoulder_limit_reached){//motor rotates CCW 
        ShoulderStepper.setSpeed(200);
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
      //ShoulderStepper.runToNewPosition(-6000);//??? steps from the limit switch gets us very close to the zero point on the arm
      //~24.44 steps per degree
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
        elbow_limit_reached = digitalRead(ELBOW_LIMIT); //this pin stays low untill limit switch is hit 
      } 
      
      while(elbow_limit_reached){ //this runs when the limit switch is being held 
        ElbowStepper.setSpeed(-100);
        ElbowStepper.runSpeed();
        elbow_limit_reached = digitalRead(ELBOW_LIMIT); 
      }
      
      ElbowStepper.setCurrentPosition(elbow_motor_position); //motor is set to zero and motor speed is zero
      ElbowStepper.setAcceleration(300);
      //ElbowStepper.runToNewPosition();//Zero point after calibration
      
      ElbowStepper.setCurrentPosition(elbow_motor_position);  //zero position is when the two arrows are lined up 
      elbow_home_set = true;
      ElbowStepper.disableOutputs();
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
        wrist_limit_reached = digitalRead(WRIST_LIMIT); //this pin stays low untill limit switch is hit 
      } 
      
      while(wrist_limit_reached){ //this runs when the limit switch is being held 
        WristStepper.setSpeed(-100);
        WristStepper.runSpeed();
        wrist_limit_reached = digitalRead(WRIST_LIMIT); 
      }
      
      WristStepper.setCurrentPosition(wrist_motor_position); //motor is set to zero and motor speed is zero
      WristStepper.setAcceleration(300);
      //WristStepper.runToNewPosition();//Zero point after calibration
      
      WristStepper.setCurrentPosition(wrist_motor_position);  //zero position is when the two arrows are lined up 
      wrist_home_set = true;
      WristStepper.disableOutputs();
    }

  }
//

/*----- AccelStepper Drivier Init ------*/
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

  //Wrist
   pinMode(WRIST_LIMIT, INPUT);
   WristStepper.setMaxSpeed(WRIST_MAX_SPEED);
   WristStepper.setEnablePin(WRIST_ENA);
   WristStepper.setPinsInverted(false, false, true);
   WristStepper.disableOutputs(); 
   

  
}
