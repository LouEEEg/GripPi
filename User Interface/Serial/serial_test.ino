void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}
    
void loop() {
  digitalWrite(LED_BUILTIN, LOW);
  
  if (Serial.available() > 0) {
    String user_command = Serial.readStringUntil('\n');

    switch (user_command.toInt()) {
      case 21:
        Serial.println("You sent value: ");
        Serial.println(user_command);
        Serial.println("Home GripPi");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        break;
      case 12:
        Serial.println("You sent value: ");
        Serial.println(user_command);
        Serial.println("Retrieve Bin 1");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        break;
      default:
        Serial.println("You sent value: ");
        Serial.println(user_command);
        Serial.println("No command Assigned");
        break;
  }
  

}
