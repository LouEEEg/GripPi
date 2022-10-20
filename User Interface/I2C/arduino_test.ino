#include <Wire.h>

#define PI_I2C_ADDR 0x44
#define GRIPPI_ADDR 0x15

static byte pi_data = 0x00; 

void setup(){
  Wire.begin(GRIPPI_ADDR);                // join i2c bus with address 0x33
  Serial.begin(9600);                     // start serial for output
  Wire.onReceive(receiveEvent);
}

void loop(){ 
  
}

void receiveEvent(int howMany)
{
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
}
