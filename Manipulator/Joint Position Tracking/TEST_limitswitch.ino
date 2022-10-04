/*
 * Written by:  Louis Kraft for EEE193 GripPi Project
 * 
 * Description: Test code for limit switch/ hall effect sensor circuit. Switches are normally closed, open at the limit.
 * 
 * 
 */

const byte J1_LIMIT_PIN = 10; 

void ISR_J1_LIMIT(void){
  digitalWrite(LED_BUILTIN, LOW);    
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(J1_LIMIT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(J1_LIMIT_PIN), ISR_J1_LIMIT, LOW); 
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   
}
