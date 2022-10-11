#include <AccelStepper.h>
#include <MultiStepper.h>

// Program Constants
#define DRIVER_INTERFACE 1     // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
#define BASE_MAX_SPEED 2000

// Joint Pin Declarations
#define BASE_LIMIT 3          
#define BASE_ENA 8             
#define BASE_PUL 2
#define BASE_DIR 7

// Global Variables
static long base_motor_position = 0;

// Interrupt Logic variables
static bool base_limit_reached = false;

// AccelStepper Object Instantiation
AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);

// ----- SETUP -----
void setup(){
  Serial.begin(9600);
  driverInit();
  attachInterrupt(digitalPinToInterrupt(BASE_LIMIT),isrBaseLimit, LOW); 
}

// ----- MAIN -----
void loop(){ 
  homeBaseAxis();
  Serial.print(base_motor_position);
}

// ----- Home Axis Algorithm
void homeBaseAxis(void){  
  bool base_home_set = false;
  BaseStepper.enableOutputs();

  while(!base_home_set){

    while(!base_limit_reached){
      BaseStepper.move(-1);    
    } 

    while(base_limit_reached){
      BaseStepper.move(1);
    } 

    base_home_set = true;
    BaseStepper.setCurrentPosition(base_motor_position);
  }

  BaseStepper.disableOutputs();
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
  base_limit_reached = true;
}
