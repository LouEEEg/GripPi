#include <AccelStepper.h>
#include <MultiStepper.h>

//Uploaded OCT 12 2022

// Program Constants
#define DRIVER_INTERFACE 1     // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
#define BASE_MAX_SPEED 2000
#define BASE_RUN_SPEED 500

// Joint Pin Declarations
#define BASE_LIMIT 3          
#define BASE_ENA 8             
#define BASE_PUL 2
#define BASE_DIR 7

// Global Variables
static long base_motor_position = 0;

// Interrupt Logic variables
// static bool base_limit_reached = false;

// AccelStepper Object Instantiation
AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);

// ----- SETUP -----
void setup(){
  Serial.begin(9600);
  driverInit();
  // attachInterrupt(digitalPinToInterrupt(BASE_LIMIT),isrBaseLimit, LOW); 
  homeBaseAxis();


}

// ----- MAIN -----
void loop(){ 
  //Serial.print(base_motor_position); 
  BaseStepper.enableOutputs();
  BaseStepper.setSpeed(-400);
  //BaseStepper.run();
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
    BaseStepper.setAcceleration(150);
    BaseStepper.runToNewPosition(-6000); //-6000 steps from the limit switch gets us very close to the zero point on the arm
    //~44.44 steps per degree
    BaseStepper.setCurrentPosition(base_motor_position);
    BaseStepper.setAcceleration(150);
    BaseStepper.runToNewPosition(-4000);
    BaseStepper.runToNewPosition(2000);    
    //BaseStepper.runToNewPosition(5);
    base_home_set = true;
    BaseStepper.disableOutputs();
  }

  
}

// ----- AccelStepper Drivier Init -----
void driverInit(void){
  // Base 
  pinMode(BASE_LIMIT, INPUT);
  BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
  BaseStepper.setEnablePin(BASE_ENA);
  BaseStepper.setPinsInverted(false, false, true);
  BaseStepper.disableOutputs(); 

   // Joint 1....
}

// ----- ISR's -----
void isrBaseLimit(void){
  //base_limit_reached = true;
}
