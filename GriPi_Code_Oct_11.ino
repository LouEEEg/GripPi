#include <AccelStepper.h>
#include <MultiStepper.h>

// Program Constants
#define DRIVER_INTERFACE 1     // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
#define BASE_MAX_SPEED 2000

// Joint Pin Declarations
#define BASE_LIMIT 3           // Base limit switch Pin
#define BASE_ENA 0             // Base enable pin
#define BASE_PUL 2
#define BASE_DIR 7

// Global Variables
volatile static float motor_speed = 0.0;

// AccelStepper Object Instantiation
AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);

void setup(){
  Serial.begin(9600);

  pinMode(BASE_LIMIT, INPUT);

  BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
  BaseStepper.setEnablePin(BASE_ENA);
  BaseStepper.setSpeed(motor_speed);
 
  attachInterrupt(digitalPinToInterrupt(BaseLimit_isr),BaseLimit_isr, LOW); 
}

void loop(){ 

  if(motor_speed == 0){
    Serial.print("Motor Speed set to: ")
    Serial.println(motor_Speed);
    motor_speed = 500.0;
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
  }

}

// ----- ISR's -----
void BaseLimit_isr(void){
  digitalWrite(LED_BUILTIN, HIGH);
  motor_speed = 0.0;
}
