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
volatile static float base_motor_speed = 500;
static float base_motor_position = 0;

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
  BaseStepper.enableOutputs(); 
  BaseStepper.setSpeed(base_motor_speed);
  BaseStepper.runSpeed(); 
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
  BaseStepper.disableOutputs(); 
  base_motor_speed = -base_motor_speed;
}
