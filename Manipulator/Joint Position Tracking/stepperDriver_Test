// Motor Driver Inputs. Jn = Joint number.
// PUL = Pulse Frequency | DIR = Direction | ENA = Enable, LOW
#define J1_PUL 8
#define J1_DIR 7
#define J1_ENA 6

void setup() {
  Serial.begin(9600);
  pinMode(J1_PUL, OUTPUT);
  pinMode(J1_DIR, OUTPUT);
  pinMode(J1_ENA, OUTPUT);


  digitalWrite(J1_DIR, LOW);
  digitalWrite(J1_ENA, LOW);
}

void loop() {
  setFrequency(1000);
}

uint32_t t1, t2 = 0;

void setFrequency(uint32_t frequency){
  uint32_t period = (uint32_t)round((1/(float)frequency)*1000000); // Period of pulse frequency in us

  digitalWrite(J1_PUL, HIGH);
  t1 = micros();

  do{
  t2 = micros();  
  }while((t2-t1) < (uint32_t)period/2); //this will accumulate error, do it better

  digitalWrite(J1_PUL, LOW);
  t1 = micros();

  do{
  t2 = micros();  
  }while((t2-t1) < (uint32_t)period/2);
  
}
