#include <Servo.h>

#define GRIPPER_PIN 9
#define GRIPPER_ANGLE_GRASP_BIN 145
#define GRIPPER_ANGLE_ENTRY_BIN 110
#define GRIPPER_ANGLE_CLOSE 175
#define GRIPPER_ANGLE_OPEN 90

Servo gripper;  

void setup() {
  gripper.attach(GRIPPER_PIN);  // attaches the servo on pin 9 to the servo object
  gripper.write(GRIPPER_ANGLE_ENTRY_BIN);
}

void loop(){
  
}
