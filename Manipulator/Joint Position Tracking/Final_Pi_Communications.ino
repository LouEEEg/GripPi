#include <GripPiList.h>
#include <GripPi.h>
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#define PI 3.14159
/*
//------Declarations------//
  // Program Constants
  //------base------//
    #define DRIVER_INTERFACE 1  // AccelStepper interface function. 1 = stepper driver with PUL/DIR pins
    #define BASE_MAX_SPEED 2000
    #define BASE_RUN_SPEED 500
    #define BASE_ACCELERATION 300
  //------shoulder------//
    #define SHOULDER_MAX_SPEED 1600
    #define SHOULDER_RUN_SPEED 500
    #define SHOULDER_ACCELERATION 200
  //------elbow------//
    #define ELBOW_MAX_SPEED 12000
    #define ELBOW_RUN_SPEED 800
    #define ELBOW_ACCELERATION 1800
  //------forarm------//
    #define FORARM_MAX_SPEED 2000
    #define FORARM_RUN_SPEED 500
    #define FORARM_ACCELERATION 200
  //------wrist------//
    #define WRIST_MAX_SPEED 2000
    #define WRIST_RUN_SPEED 500
    #define WRIST_ACCELERATION 300
//------Joint Declarations------//
  // Base Joint Declarations //Driver #1
    #define BASE_LIMIT 10  //standard pin  GREEN
    #define BASE_ENA 11    //standard pin  WHITE
    #define BASE_PUL 12    //standard pin  ORANGE
    #define BASE_DIR 13    //PWM pin YELLOW

  //Shoulder joint Declarations  //Driver #2&3
    #define SHOULDER_LIMIT 52  //standard pin  GREEN     (pull down circuit)
    #define SHOULDER_ENA 4     //standard pin (was pwm)  WHITE
    #define SHOULDER_PUL 5     //standard pin (was pwm)  ORANGE
    #define SHOULDER_DIR 6     //PWM pin YELLOW

  //Elbow joint Declarations //Driver #4
    #define ELBOW_LIMIT 51  //standard pin
    #define ELBOW_ENA 32    //standard pin
    #define ELBOW_PUL 33    //standard pin
    #define ELBOW_DIR 34    //PWM pin

  //Forarm Rotation Declarations //Driver #5
    #define FORARM_LIMIT 50
    #define FORARM_ENA 9 //White
    #define FORARM_PUL 8 //Orange
    #define FORARM_DIR 7 //Yellow

  //Wrist Joint Declarations //Driver #6
    #define WRIST_LIMIT 49  //standard pin
    #define WRIST_ENA 47    //standard pin
    #define WRIST_PUL 46    //standard pin
    #define WRIST_DIR 45    //PWM pin
*/
//------Global Variables------//
  Servo Servo_Grip;
  static long base_motor_position = 0;
  static long shoulder_motor_position = 0;
  static long elbow_motor_position = 0;
  static long forarm_motor_position = 0;
  static long wrist_motor_position = 0;
  int base_steps = 0;
  int shoulder_steps = 0;
  int elbow_steps = 0;
  int forarm_steps = 0;
  int wrist_steps = 0;
  int grip_pos = 90;
  bool base_location_reached = 0;
  bool shoulder_location_reached = 0;
  bool elbow_location_reached = 0;
  bool forarm_location_reached = 0;
  bool wrist_location_reached = 0;
  int current_base_pos;
  int current_shoulder_pos;
  int current_elbow_pos;
  int current_wrist_pos;
  float PosX = 0;
  float PosY = 0;
  float PosZ = 0;



using namespace BLA;
//------limb angles in rad------//
  float theta0 = 0; //
  float theta1 = 0; // 
  float theta2 = 0; //
  float theta3 = 0; // 

//------limb length in milimeters------//
  float a0 = 221; //base to shoulder
  float a1 = 221; //shoulder to elbow
  float a2 = 218; //elbow to wrist
  float a3 = 147; //wrist to gripper

//------Position Declarations------//
//temp inputs
  float Pxd = 0; 
  float Pyd = 0; // zero point is when arm is vertical 
  float Pzd = 0; 

  float distance = 0; 
  float APx = 0;
  float APy = 0;

//------AccelStepper Object Instantiation------//
  AccelStepper BaseStepper(DRIVER_INTERFACE, BASE_PUL, BASE_DIR);
  AccelStepper ShoulderStepper(DRIVER_INTERFACE, SHOULDER_PUL, SHOULDER_DIR);
  AccelStepper ElbowStepper(DRIVER_INTERFACE, ELBOW_PUL, ELBOW_DIR);
  AccelStepper ForarmStepper(DRIVER_INTERFACE, FOREARM_PUL, FOREARM_DIR);
  AccelStepper WristStepper(DRIVER_INTERFACE, WRIST_PUL, WRIST_DIR);
//
//------stolen test code------//
  const byte numChars = 32;
  char receivedChars[numChars];
  char tempChars[numChars];        // temporary array for use when parsing

        // variables to hold the parsed data
  float PCX = 0;
  float PCY = 0;
  float PCZ = 0;
  float PCG = 130;

boolean newData = false;
//------ end of code------//

//------linked list declaraitions------//
  BLA::Matrix<5, 4> BinOpen1={420,300,270,90, 485,210,315,90, 0,0,0,0, 0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> BinOpen2={354,300,108,90, 402,250,122,90, 485,210,148,90, 0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> BinOpen3={370,300,0,90, 420,250,0,90, 485,210,0,90,  0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> BinToUser1={485,270,315,130, 420,300,270,130, 202,300,310,130, 1,120,350,130, 1,70,340,130};
  BLA::Matrix<5, 4> BinToUser2={485,270,148,130, 354,300,108,130, 127,300,348,130, 1,120,350,130, 1,70,340,130};
  BLA::Matrix<5, 4> BinToUser3={485,270,0,130, 370,300,0,130, 127,300,348,130, 1,120,350,130, 1,70,340,130};
  BLA::Matrix<5, 4> UserToCamera={1,70,345,90, 1,95,350,130, 1,150,350,130, 300,70,300,130, 0,0,0,0};
  BLA::Matrix<5, 4> CameraToBin1={200,130,300,130, 200,230,346,130, 310,300,201,130, 485,270,315,130, 485,210,315,130};
  BLA::Matrix<5, 4> CameraToBin2={200,130,300,130, 200,230,346,130, 354,300,108,130, 485,270,148,130, 490,210,150,130};
  BLA::Matrix<5, 4> CameraToBin3={200,130,300,130, 200,230,346,130, 370,300,0,130, 485,270,0,130, 485,210,0,130};
  BLA::Matrix<5, 4> BinToNeutral1={485,300,315,90, 420,300,270,90, 300,300,350,90, 0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> BinToNeutral2={485,300,148,90, 354,300,108,90, 300,300,350,90, 0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> BinToNeutral3={485,300,0,90, 370,300,0,90, 300,300,350,90, 0,0,0,0, 0,0,0,0};
  BLA::Matrix<5, 4> temp;

  BLA::Matrix<4, 4> TOTAL;
//
  //float temp [5][4]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //float BinOpen1 [2][4]= {{420, 300, 270, 90},{485, 210, 320, 90},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //float BinOpen2 [2][4]= {{354, 300, 108, 90},{485, 210, 148, 90},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //float BinOpen3 [2][4]= {{370, 300, 0, 90},{485, 210, 0, 90},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //float BinToUser1 [5][4]={{485, 300, 315, 130},{420, 300, 270, 130},{202, 300, 310, 130},{1, 120, 350, 130},{1, 70, 345, 130}};
  //float BinToUser2 [5][4]={{485, 300, 148, 130},{354, 300, 108, 130},{127, 300, 348, 130},{1, 120, 350, 130},{1, 70, 345, 130}};
  //float BinToUser3 [5][4]={{485, 300, 0, 130},{370, 300, 0, 130},{127, 300, 348, 130},{1, 120, 350, 130},{1, 70, 345, 130}};
  //float UserToCamera [5][4]={{1, 95, 350, 130},{1, 120, 350, 130},{300, 70, 300, 130},{0,0,0,0},{0,0,0,0}};
  //float CameraToBin1 [5][4]={{200, 95, 300, 130},{420, 270, 270, 130},{485, 270, 320, 130},{485, 210, 320, 130},{0,0,0,0}};
  //float CameraToBin2 [5][4]={{200, 95, 300, 130},{354, 300, 108, 130},{485, 270, 148, 130},{485, 210, 148, 130},{0,0,0,0}};
  //float CameraToBin3 [5][4]={{200, 95, 300, 130},{370, 300, 0, 130},{485, 300, 0, 130},{485, 210, 0, 130},{0,0,0,0}};
  //float BinToNeutral1 [5][4]={{485, 300, 320, 90},{420, 300, 270, 90},{300, 300, 350, 90},{0,0,0,0},{0,0,0,0}};
  //float BinToNeutral2 [5][4]={{485, 300, 148, 90},{354, 300, 108, 90},{300, 300, 350, 90},{0,0,0,0},{0,0,0,0}};
  //float BinToNeutral3 [5][4]={{485, 300, 0, 90},{370, 300, 0, 90},{300, 300, 350, 90},{0,0,0,0},{0,0,0,0}};

  int length=0;


void setup() {
  //SerialUSB.begin(115200);
  //SerialUSB.begin(115200);
  //InverseKinematics();
  //ForwardKinematics();
  SerialUSB.begin(9600);  // Initialize Native USB port


  driverInit(); //declare driver parameters
  BaseStepper.enableOutputs();
  ShoulderStepper.enableOutputs();
  ElbowStepper.enableOutputs();
  ForarmStepper.enableOutputs();
  WristStepper.enableOutputs();
  //WakeUp(); //test of the wake up sequence
  //homeBaseAxis(); //test of each individual calibration sequences
  //homeShoulderAxis();
  //homeElbowAxis();
  //homeForarmAxis();

  //----test code----//
    SerialUSB.println("This demo expects 4 pieces of data - X, Y, Z and grip values");
    SerialUSB.println("Enter data in this style <X value, Y value, Z value Grip Value (90 or 130)>");
    SerialUSB.println();
}

void loop() {
  //------test code------// 
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            // because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        ForwardKinematics();
        
    //==========Sequencing==========//
      //==========Home Sequences==========//_
        if(PCX==999){ 
          WakeUp();
          SerialUSB.print("GripPi is Calibrated");
          Pxd = 127;
          Pyd = 300; 
          Pzd = 348; 
          InverseKinematics();
          DegToSteps();
          Servo_Grip.write(90);

          newData = false;
        }
        else if(PCX==998){
          ReCalibrate();
          SerialUSB.print("GripPi is Calibrated");
          Pxd = 127;
          Pyd = 300; 
          Pzd = 348; 
          Servo_Grip.write(90);
          InverseKinematics();
          DegToSteps();      
          newData = false;
        }
        
      //==========move to bin 1 from safe place==========//
        else if(PCX == 485 && PCY == 210 && PCZ == 315 && PCG ==90 ){//bin 1 position open
          //linked list sequence to safely aquire bin 1 
          length=2;  
          temp = BinOpen1;
          ArrayCycle();
          newData = false;
        }
        
      //==========move to bin 2 from safe place==========//
        else if(PCX == 485 && PCY == 210 && PCZ ==148 && PCG ==90){//bin 2 position open
          //linked list sequence to safely aquire bin 2
          length=3;
          temp = BinOpen2;
          ArrayCycle();
          newData = false;
        }

      //==========move to bin 3 from safe place==========//
        else if(PCX == 485 && PCY == 210 && PCZ ==0 && PCG ==90){ //bin 3 position open
          //linked list sequence to safely aquire bin 3 and return to the safe location 
          length=3;
          temp = BinOpen3;        
          ArrayCycle();
          newData = false;
        }

      //==========bin to user==========//
        else if(PCX == 1 && PCY == 70 && PCZ ==345 && PCG ==130){ //user location closed 
          //linked list sequence to go to the user location and return to the safe location
          //ForwardKinematics();
          //SerialUSB.println(PosZ);
          if(PosZ == 315 ){ //bin1 
            //linked list creation
            length=5;
            temp = BinToUser1; 
          ArrayCycle();
          newData = false;
          }

          if(PosZ == 148 ){ //bin2
            //linked list creation
            length=5;
            temp = BinToUser2; 
          ArrayCycle();
          newData = false;
          }

          if(PosZ == 0 ){ //bin3
            //linked list creation 
            length=5;
            temp = BinToUser3;
            ArrayCycle();
            newData = false;
          }
          else{ //if not at a bin
            Pxd = PCX; 
            Pyd = PCY; 
            Pzd = PCZ;
            PositionCycle();
            newData = false;
          }
        }

      //==========user to camera==========//
        else if(PCX == 300 && PCY == 70 && PCZ ==300 && PCG == 130){ //camera location closed
          //linked list 
          length=4;
          temp = UserToCamera;
          ArrayCycle(); 
          newData = false; 
          } 
      //==========camera to bin==========//
        else if(PCX == 485 && PCY == 210 && PCZ == 315 && PCG ==130 ){//camera to bin 1
          //linked list sequence to safely move bin 1 from camera to its place
          length=5;
          temp = CameraToBin1;
          ArrayCycle();
          newData = false;
        }

        else if(PCX == 485 && PCY == 210 && PCZ ==148 && PCG ==130 ){//camera to bin 2
          //linked list sequence to safely move bin 2 from camera to its place
          length=5;
          temp = CameraToBin2;
          ArrayCycle();
          newData = false;
        }

        else if(PCX == 485 && PCY == 210 && PCZ ==0 && PCG ==130 ){//camera to bin 3
          //linked list sequence to safely move bin 3 from camera to its place
          length=5;
          temp = CameraToBin3;
          ArrayCycle();
          newData = false;
        }

      //==========return to idle position==========//
        else if  (PCX == 127 && PCY == 300 && PCZ ==348 && PCG ==90){ //neutral idle position
          //ForwardKinematics();
          if(PosZ == 315 ){ //bin1 
            //linked list creation 
            length=3;
            temp = BinToNeutral1;
            ArrayCycle();
            newData = false;
            }

          else if(PosZ == 150 ){ //bin2
            //linked list creation
            length=3;
            temp = BinToNeutral2; 
            ArrayCycle();
            newData = false;
            }

          else if(PosZ == 0 ){ //bin3
            //linked list creation
            length=3;
            temp = BinToNeutral3;
            ArrayCycle();
            newData = false;
            }
          else{
            Pxd = PCX; 
            Pyd = PCY; 
            Pzd = PCZ;
            PositionCycle();
            newData = false;
            SerialUSB.println("GripPi has moved");
            
            }
        
        }

     
        else{
          Pxd = PCX; 
          Pyd = PCY; 
          Pzd = PCZ;
          PositionCycle();
          newData = false;
          SerialUSB.println("GripPi has moved");
            }
    }
  Navigation();
}
//------Navigation------//
  void Navigation(){
    //------loop for when the location has already been reached------//
      BaseStepper.enableOutputs();
      ShoulderStepper.enableOutputs();
      ElbowStepper.enableOutputs();
      ForarmStepper.enableOutputs();
      WristStepper.enableOutputs();

    //-----------base inputs----------//
      BaseStepper.moveTo(base_steps);  //the pi will update this value base_steps
      BaseStepper.setAcceleration(BASE_ACCELERATION);
      BaseStepper.run();

    //---------shoulder inputs-------//
      ShoulderStepper.moveTo(shoulder_steps);  //the pi will update this value shoulder_steps
      ShoulderStepper.setAcceleration(SHOULDER_ACCELERATION);
      ShoulderStepper.run();

    //------Elbow Inputs------//
      ElbowStepper.moveTo(elbow_steps);  //the pi will update this value elbow_steps
      ElbowStepper.setAcceleration(ELBOW_ACCELERATION);
      ElbowStepper.run();

    //------Wrist Inputs------//
      WristStepper.moveTo(wrist_steps);  //the pi will update this value wrist_steps
      WristStepper.setAcceleration(WRIST_ACCELERATION);
      WristStepper.run();

      
    //------checking for input changes//
      base_location_reached = BaseStepper.distanceToGo();
      shoulder_location_reached = ShoulderStepper.distanceToGo();
      elbow_location_reached = ElbowStepper.distanceToGo();
      forarm_location_reached = ForarmStepper.distanceToGo();
      wrist_location_reached = WristStepper.distanceToGo();
    
    //-------While loop to run while joints are moving------//
    while (base_location_reached != 0 || shoulder_location_reached != 0 || elbow_location_reached != 0 || wrist_location_reached != 0) {
      //once the inputs are recieved, this loop plays traveling to the location
      BaseStepper.enableOutputs();
      ShoulderStepper.enableOutputs();
      ElbowStepper.enableOutputs();
      ForarmStepper.enableOutputs();
      WristStepper.enableOutputs();
      base_location_reached = BaseStepper.distanceToGo();
      BaseStepper.run();
      shoulder_location_reached = ShoulderStepper.distanceToGo();
      ShoulderStepper.run();
      elbow_location_reached = ElbowStepper.distanceToGo();
      ElbowStepper.run();
      wrist_location_reached = WristStepper.distanceToGo();
      WristStepper.run();
    }  
  } 

  void PositionCycle(){
    //------cycle the linked list positions------//
      Servo_Grip.write(PCG);
      InverseKinematics();
      DegToSteps();
      Navigation();
    } 



  //------cycling and running each linked point------//
  void ArrayCycle(){
    for(int i=0; i<length; i++){
        Pxd=temp(i,0);
        Pyd=temp(i,1);
        Pzd=temp(i,2);
        PositionCycle();                

    }
        SerialUSB.println("GripPi has moved");
  }

//------matrix math------// 
  void ForwardKinematics(){

    //caclulate the homogeneous transformations for the movement in reference to the starting position
    //setting up the matricies  
    BLA::Matrix<4, 4> BASE =        {cos(theta0),0,-sin(theta0),0,0,1,0,0,sin(theta0),0,cos(theta0),0,0,0,0,1}; //rortate about the Y
    BLA::Matrix<4, 4> BASETR =      {1,0,0,0,0,1,0,a0,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> SHOULDER =    {cos(theta1),-sin(theta1),0,0,sin(theta1),cos(theta1),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> SHOULDERTR =  {1,0,0,0,0,1,0,a1,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> ELBOW =       {cos(theta2),-sin(theta2),0,0,sin(theta2),cos(theta2),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> ELBOWTR=      {1,0,0,0,0,1,0,a2,0,0,1,0,0,0,0,1};//Shift along the Y
    BLA::Matrix<4, 4> WRIST =       {cos(theta3),-sin(theta3),0,0,sin(theta3),cos(theta3),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
    BLA::Matrix<4, 4> WRISTTR=      {1,0,0,0,0,1,0,a3,0,0,1,0,0,0,0,1};//Shift along the Y

    //-----------------------------------------------------------------------------------------------------//
    
    //doing the matrix math   
    BLA::Matrix<4, 4> BASETOTAL =     BASE*BASETR;
    BLA::Matrix<4, 4> SHOULDERTOTAL = SHOULDER*SHOULDERTR;
    BLA::Matrix<4, 4> ELBOWTOTAL =    ELBOW*ELBOWTR;
    BLA::Matrix<4, 4> WRISTTOTAL =    WRIST*WRISTTR;

    TOTAL=BASETOTAL*SHOULDERTOTAL*ELBOWTOTAL*WRISTTOTAL;
  
    //calculate the new position matrix
    //BLA::Matrix<4> POSF = {TOTAL(0,3),TOTAL(1,3),TOTAL(2,3),0}; //what is calculated each time 

    //*need to change to inherit previous calc*

    //these are to display the various values useful for checking math
    //Serial << "POSF: " << POSF << '\n';       
    //Serial << "POSFX: " << TOTAL(0,3) << '\n';
    //Serial << "POSFY: " << TOTAL(1,3) << '\n';
    //Serial << "POSFZ: " << TOTAL(2,3) << '\n';

    PosX = TOTAL(0,3);
    PosY = TOTAL(1,3);
    PosZ = TOTAL(2,3);
    PosZ = roundf(PosZ); 


    //Serial << "BASETOTAL: " << BASETOTAL << '\n';
    //Serial << "SHOULDERTOTAL: " << SHOULDERTOTAL << '\n';
    //Serial << "ELBOWTOTAL: " << ELBOWTOTAL << '\n';
    //Serial << "WRISTTOTAL: " << WRISTTOTAL << '\n';

    //Serial << "TOTAL: " << TOTAL << '\n';


  }

  void InverseKinematics(){
    //------calculating theta 0, theta 1, theta 2, and theta3------//
    distance=sqrt(sq(Pzd)+sq(Pxd));   //calculate distance from Yaxis to Desired point 
    theta0=atan(Pzd/Pxd);             //calculate theta 0 to align axis with the distance 
    APx=distance-a3;                  //adjusted x axis 
    APy=Pyd-a0;                       //adjusted y axis 
    theta2=acos((sq(APx)+sq(APy)-sq(a1)-sq(a2))/(2*a1*a2));   //calculate theta 2 
    theta1=atan(APy/APx)+atan((a2*sin(theta2))/(a1+a2*cos(theta2))); //calculate theta 1 

    //------compensating for the orientation of our manipulator------//
    theta1=-(PI/2-theta1); 
    theta2=-theta2; 
    theta3=-(PI/2+theta1+theta2); 
     /*
    SerialUSB.print("theta0: ");
    SerialUSB.println(theta0);
    SerialUSB.print("theta1: ");
    SerialUSB.println(theta1);
    SerialUSB.print("theta2: ");
    SerialUSB.println(theta2);
    SerialUSB.print("theta3: ");
    SerialUSB.println(theta3);
    */
  }
  void DegToSteps(){//convert the new thetas into steps the system looks for 
    base_steps=44.444*theta0*180/PI;
    shoulder_steps=-24.4*theta1*180/PI;
    elbow_steps= -100*theta2*180/PI;
    wrist_steps=19.44*theta3*180/PI;
    /*
    SerialUSB.print("base_steps: ");
    SerialUSB.println(base_steps);
    SerialUSB.print("shoulder_steps: ");
    SerialUSB.println(shoulder_steps);
    SerialUSB.print("elbow_steps: ");
    SerialUSB.println(elbow_steps);
    SerialUSB.print("wrist_steps: ");
    SerialUSB.println(wrist_steps);
    */

  }
//------calibrations------//
  void WakeUp(void){
      BaseStepper.enableOutputs();
      ShoulderStepper.enableOutputs();
      ElbowStepper.enableOutputs();
      ForarmStepper.enableOutputs();
      WristStepper.enableOutputs();
      
      ShoulderStepper.setAcceleration(200);
      ShoulderStepper.runToNewPosition(-2000); //test //shoulder moves back about 80-90 degrees
      //BaseStepper.setAcceleration(300);
      //BaseStepper.runToNewPosition(-4000); //test
      ElbowStepper.setAcceleration(800);
      ElbowStepper.runToNewPosition(-9000); //test //elbow moves back about 90 degrees

      homeShoulderAxis();
      homeElbowAxis();
      homeWristAxis();
      homeBaseAxis();
  }

  void ReCalibrate(void){
    homeWristAxis();
    homeElbowAxis();    
    homeShoulderAxis();    
    homeBaseAxis();
    //BaseStepper.setAcceleration(300);
    //BaseStepper.runToNewPosition(4000); //test
    //BaseStepper.setAcceleration(300);
    //BaseStepper.runToNewPosition(0);
    
  }

  /*----- Base Axis Calibration------*/
  void homeBaseAxis(void) {
    bool base_home_set = false;
    int base_limit_reached = 0;

    while (!base_home_set) {

      while (!base_limit_reached) {  //motor rotates CCW
        BaseStepper.setSpeed(500);
        BaseStepper.runSpeed();
        base_limit_reached = digitalRead(BASE_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (base_limit_reached) {  //this runs when the limit switch is being held
        BaseStepper.setSpeed(-100);
        BaseStepper.runSpeed();
        base_limit_reached = digitalRead(BASE_LIMIT);
      }

      BaseStepper.setCurrentPosition(base_motor_position);  //motor is set to zero and motor speed is zero
      BaseStepper.setAcceleration(300);
      BaseStepper.runToNewPosition(-6000);  //-6000 steps ~135deg from the limit switch gets us very close to the zero point on the arm
      //~44.44 steps per degree     //-4000~90deg
      BaseStepper.setCurrentPosition(base_motor_position);  //zero position is when the two arrows are lined up
      base_home_set = true;
    }
  }

  /*----- Shoulder Axis Calibration------*/
  void homeShoulderAxis(void) {
    bool shoulder_home_set = false;
    int shoulder_limit_reached = 0;

    while (!shoulder_home_set) {

      while (!shoulder_limit_reached) {  //Joint rotates forwards
        ShoulderStepper.setSpeed(200);
        ShoulderStepper.runSpeed();
        shoulder_limit_reached = !digitalRead(SHOULDER_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (shoulder_limit_reached) {  //this runs when the limit switch is being held
        ShoulderStepper.setSpeed(-100);
        ShoulderStepper.runSpeed();
        shoulder_limit_reached = !digitalRead(SHOULDER_LIMIT);
      }

      ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //motor is set to zero and motor speed is zero
      ShoulderStepper.setAcceleration(200);
      ShoulderStepper.runToNewPosition(-2200);//+ = forwards, - = backwards 2200 steps ~90 degrees
      //~24.44 steps per degree
      ShoulderStepper.setCurrentPosition(shoulder_motor_position);  //zero position is when the two arrows are lined up

      shoulder_home_set = true;
    }
  }

  //------Elbow Axis Calibration------//
  void homeElbowAxis(void) {
    bool elbow_home_set = false;
    int elbow_limit_reached = 0;

    while (!elbow_home_set) {

      while (!elbow_limit_reached) {

        ElbowStepper.setSpeed(1500);  //Arm rotates forwards
        ElbowStepper.runSpeed();
        elbow_limit_reached = !digitalRead(ELBOW_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (elbow_limit_reached) {  //this runs when the limit switch is being held
        ElbowStepper.setSpeed(-1000);
        ElbowStepper.runSpeed();
        elbow_limit_reached = !digitalRead(ELBOW_LIMIT);
      }

      ElbowStepper.setCurrentPosition(elbow_motor_position);  //motor is set to zero and motor speed is zero
      ElbowStepper.setAcceleration(800);
      ElbowStepper.runToNewPosition(-8400);//+ = forwards, - = backwards 9000 ~90deg

      ElbowStepper.setCurrentPosition(elbow_motor_position);  //zero position is when the two arrows are lined up

      elbow_home_set = true;
    }
  }

  //------Calibration Sequence for the Wrist------//
  void homeWristAxis(void) {
    bool wrist_home_set = false;
    int wrist_limit_reached = 0;


    while (!wrist_home_set) {

      while (!wrist_limit_reached) {

        WristStepper.setSpeed(-300);  //Arm rotates forwards
        WristStepper.runSpeed();
        wrist_limit_reached = !digitalRead(WRIST_LIMIT);  //this pin stays low untill limit switch is hit
      }

      while (wrist_limit_reached) {  //this runs when the limit switch is being held
        WristStepper.setSpeed(100);
        WristStepper.runSpeed();
        wrist_limit_reached = !digitalRead(WRIST_LIMIT);
      }

      WristStepper.setCurrentPosition(wrist_motor_position);  //motor is set to zero and motor speed is zero
      WristStepper.setAcceleration(300);
      WristStepper.runToNewPosition(1800);//- = forwards, + = backwards 1750 steps ~90 degrees

      WristStepper.setCurrentPosition(wrist_motor_position);  //zero position is when the two arrows are lined up
      wrist_home_set = true;
      
    }
  }

/*----- AccelStepper Drivier Init ------*/
  void driverInit(void) {
    // Base
    pinMode(BASE_LIMIT, INPUT);
    BaseStepper.setMaxSpeed(BASE_MAX_SPEED);
    BaseStepper.setEnablePin(BASE_ENA);
    BaseStepper.setPinsInverted(false, false, true);
    BaseStepper.disableOutputs();

    // Shoulder
    pinMode(SHOULDER_LIMIT, INPUT);
    ShoulderStepper.setMaxSpeed(SHOULDER_MAX_SPEED);
    ShoulderStepper.setEnablePin(SHOULDER_ENA);
    ShoulderStepper.setPinsInverted(false, false, true);
    ShoulderStepper.disableOutputs();

    // Elbow
    pinMode(ELBOW_LIMIT, INPUT);
    ElbowStepper.setMaxSpeed(ELBOW_MAX_SPEED);
    ElbowStepper.setEnablePin(ELBOW_ENA);
    ElbowStepper.setPinsInverted(false, false, true);
    //ElbowStepper.disableOutputs();

    // Forarm
    pinMode(FOREARM_LIMIT, INPUT);
    ForarmStepper.setMaxSpeed(FOREARM_MAX_SPEED);
    ForarmStepper.setEnablePin(FOREARM_ENA);
    ForarmStepper.setPinsInverted(false, false, true);
    //ForarmStepper.disableOutputs();

    //Wrist
    pinMode(WRIST_LIMIT, INPUT);
    WristStepper.setMaxSpeed(WRIST_MAX_SPEED);
    WristStepper.setEnablePin(WRIST_ENA);
    WristStepper.setPinsInverted(false, false, true);
    //WristStepper.disableOutputs();

    Servo_Grip.attach(53);
    Servo_Grip.write(90);

  }
//------serial code------//
  void recvWithStartEndMarkers() {
      static boolean recvInProgress = false;
      static byte ndx = 0;
      char startMarker = '<';
      char endMarker = '>';
      char rc;

      while (SerialUSB.available() > 0 && newData == false) {
          rc = SerialUSB.read();

          if (recvInProgress == true) {
              if (rc != endMarker) {
                  receivedChars[ndx] = rc;
                  ndx++;
                  if (ndx >= numChars) {
                      ndx = numChars - 1;
                  }
              }
              else {
                  receivedChars[ndx] = '\0'; // terminate the string
                  recvInProgress = false;
                  ndx = 0;
                  newData = true;
              }
          }

          else if (rc == startMarker) {
              recvInProgress = true;
          }
      }
  }

  //============

  void parseData() {      // split the data into its parts

      char * strtokIndx; // this is used by strtok() as an index

      strtokIndx = strtok(tempChars,",");      // get the first part
      PCX = atof(strtokIndx); // copy it to messageFromPC
  
      strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
      PCY = atof(strtokIndx);     

      strtokIndx = strtok(NULL, ",");
      PCZ = atof(strtokIndx);     

      strtokIndx= strtok(NULL, ",");
      PCG = atof(strtokIndx);
  }

  //============

  void showParsedData() {

      SerialUSB.print("X ");
      SerialUSB.println(PCX);
      SerialUSB.print("Y ");
      SerialUSB.println(PCY);
      SerialUSB.print("Z ");
      SerialUSB.println(PCZ);
      SerialUSB.print("G ");
      SerialUSB.println(PCG);
      
  }

