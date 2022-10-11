#include <AccelStepper.h>
#include <MultiStepper.h>

//connect DIR+, PUL+ and pull up resistor to interupt button to +5V power from board

//Defining the limit switches
#define BaseLimit 3 //Base Limit switch 
#define Joint1Limit 4 //Joint 1 limit switch
#define Joint2Limit 11 //Joint 2 limit switch
#define Joint3Limit 14
#define Joint4Limit 19

//Potentiometer test for case states
int PotIn = A0;
int LastPotIn;

//building the state system for each container
int state = 0;
int PiInput = 0;


//defining all the motors
//Connect PWM pin to DIR- and connect Direction pin to PUL-
AccelStepper BaseStepper(1,2,1);    //BaseStepper is connected to pins 1(PWM), 2(direction), 3(interupt)
AccelStepper JointStepper1(1,5,6);  //Joint1 is connected to pins 6(PWM), 5(direction), 4(interupt)
AccelStepper JointStepper2(1,10,9); //Joint2 is connected to pins 9(PWM), 10(Direction), 11(Interupt)
AccelStepper JointStepper3(1,12,13);//Joint3 is connected to pins 13(PWM), 12(Direction), 14(interupt)
AccelStepper JointStepper4(1,18,17);//Joint4 is connected to pins 17(Pwm), 18(direction), 19(Interupt)


void setup() 
{
  Serial.begin(9600);
  pinMode(BaseLimit, INPUT);
  pinMode(Joint1Limit, INPUT);
  pinMode(Joint2Limit, INPUT);
  pinMode(Joint3Limit, INPUT);
  pinMode(Joint4Limit, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PotIn,INPUT);

  //setting the mortors max speed and moving the motors to hit the limit switches
  BaseStepper.setMaxSpeed(1000);
  BaseStepper.setSpeed(-500);
  BaseStepper.runSpeed(); //motor direction and speed(towards the limit switch)
  /* 
  JointStepper1.setMaxSpeed(500);
  JointStepper1.setSpeed(-50); 
  JointStepper1.runSpeed();

  JointStepper2.setMaxSpeed(500);
  JointStepper2.setSpeed(-50); 
  JointStepper2.runSpeed();

  JointStepper3.setMaxSpeed(500);
  JointStepper3.setSpeed(-50); 
  JointStepper3.runSpeed();

  JointStepper4.setMaxSpeed(500);
  JointStepper4.setSpeed(-50); 
  JointStepper4.runSpeed();
  */
 

  //Interupts
  //attachInterrupt(digitalPinToInterrupt(BaseLimit),calibrateBase, FALLING); //triggers when the base limit switch goes high so we need a pull up resistor 
  //attatchinterupt(digitalPinToInterupt(Joint1Limit),calibrateJoint1,RISING);
  //attatchinterupt(digitalPinToInterupt(Joint2Limit),calibrateJoint2,RISING);
  //attatchinterupt(digitalPinToInterupt(Joint3Limit),calibrateJoint3,RISING);
  //attatchinterupt(digitalPinToInterupt(Joint4Limit),calibrateJoint4,RISING);

}
/*
void calibrateBase()//maybe we could allocate this function into an input to select when the arm forgets where it is
{
  //limit switch triggers and runs this program
  BaseStepper.move(500);//moves to our Neutral location 100 for now
  BaseStepper.setCurrentPosition(0);//sets the new current pos to 0 
  
}
 we can change this whole series into another state function in which the calibration happens one joint at a time. 
void calibrateJoint1()
{
  //limit switch triggers and runs this program
  JointStepper1.move(100);//moves to our Neutral location 100 for now
  JointStepper1.setCurrentPosition(0);//sets the new current pos to 0 
  
}

void calibrateJoint2()
{
  //limit switch triggers and runs this program
  JointStepper2.move(100);//moves to our Neutral location 100 for now
  JointStepper2.setCurrentPosition(0);//sets the new current pos to 0 
  
}

void calibrateJoint3()
{
  //limit switch triggers and runs this program
  JointStepper3.move(100);//moves to our Neutral location 100 for now
  JointStepper3.setCurrentPosition(0);//sets the new current pos to 0 
  
}
*/

void loop() 
{ 
  int val = analogRead(PotIn)/255;
  Serial.println(val);
  
  // can have a seperate pin for each state input from the pi
  // or manage to have one input write multiple states bases on the type of input
    //an idea for this is for the raspbery pi to generate a pulse and the period of the pulse decides which case is selected 

  //waiting for an analog input from the Raspberry pi (currently changing the values from 0-4)

  switch (val)
  {
    case 0:{   
      digitalWrite(LED_BUILTIN, LOW);                  
    //BaseStepper.move(whatever the angle needed)
    //JointStepper1.move(wherever)
    //JointStepper2.move(wherever)
    //JointStepper3.move(wherever)
    //JointStepper4.move(wherever)
    //reset to state zero
    }break;
    case 1:{//navigat to and from object 1
      digitalWrite(LED_BUILTIN, HIGH);  
      
   //BaseStepper.move(whatever the angle needed)
    //JointStepper1.move(wherever)
    //JointStepper2.move(wherever)
    //JointStepper3.move(wherever)
    //JointStepper4.move(wherever)
    //reset to state zero
    }break;

   case 2:{//navigate to and from object 2 
                           
      digitalWrite(LED_BUILTIN, LOW);    
      
    //BaseStepper.move(whatever the angle needed)
    //JointStepper1.move(wherever)
    //JointStepper2.move(wherever)
    //JointStepper3.move(wherever)
   //JointStepper4.move(wherever)
    //reset to state zero
    }break;

    case 3:{ //navigate to and from object 3
      digitalWrite(LED_BUILTIN, HIGH); 

    //BaseStepper.move(whatever the angle needed)
    //JointStepper1.move(wherever)
    //JointStepper2.move(wherever)
    //JointStepper3.move(wherever)
    //JointStepper4.move(wherever)
    //reset to state zero
    }break;

    case 4:{ //navigate to and from object 4
    digitalWrite(LED_BUILTIN, LOW); 
    
    //BaseStepper.move(whatever the angle needed)
    //JointStepper1.move(wherever)
    //JointStepper2.move(wherever)
    //JointStepper3.move(wherever)
    //JointStepper4.move(wherever)
    //reset to state zero
    }break;

    default:{ 
      digitalWrite(LED_BUILTIN, HIGH);

    }break;

  }
  
}


