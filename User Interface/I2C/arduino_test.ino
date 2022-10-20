#include <Wire.h>

#define PI_I2C_ADDR 0x44
#define GRIPPI_ADDR 0x15

static byte pi_data = 0x00; 

void setup(){
  Wire.begin(GRIPPI_ADDR);                // join i2c bus with address 0x33
  Serial.begin(9600);                     // start serial for output
  Wire.onReceive(receiveEvent);
}

static int angle_matrix[6] = {0,0,0,0,0,0};
static int i = 0;
int print_data = 0;

void loop(){ 
    if(print_data == 1){
      for(int d=0;d<6;d++){
        Serial.println(angle_matrix[d]);
      }
      print_data =0;
    }
}

void receiveEvent(int howMany){
  while(1 < Wire.available()){ 
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  
  int x = Wire.read();    // receive byte as an integer
  angle_matrix[i] = x;
  i++;
  if(i == 6){
    i = 0;
    print_data = 1;
  }
  
}
