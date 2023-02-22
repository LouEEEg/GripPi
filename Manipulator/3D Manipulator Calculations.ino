#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
 

#define PI 3.1415926535897932384626433832795 
using namespace BLA;
//------limb angles in deg------//
float theta0 = 0; //-Pi/2
float theta1 = 0; //
float theta2 = 0; //
float theta3 = 0; //
float theta4 = 0; //

//------limb length in milimeters------//
float a0 = 221; //base to shoulder
float a1 = 221; //shoulder to elbow
float a2 = 109; //elbow to forarm
float a3 = 109; //forarm to wrist
float a4 = 147; //wrist to gripper(will need to be extended)

//------Position Declarations------//
float Pxd = 355.6; //example bin2 location 14inch = 355.6mm
float Pyd = 304.8; //example bin2 location 12inch = 304.8mm
float Pzd = 0; 
float distance = 0; 

//------incriment levels------//
float PITHETA0=PI/5;    //10 incriments to cover a distance of PI     from PI/6 to -PI/2 (from the over bed table to the far bin location)
float PITHETA1=PI/10;   //10 incriments to cover a distance of Pi/2   from 0 to PI/2 
float PITHETA2=PI/20;   //10 increments to cover a distance of PI/2   from 0 to PI/2
float PITHETA3=PI/5;    //10 incriments to cover distance of PI       from -PI/2 to PI/2
float PITHETA4=PI*2/30; //10 incriments to cover distance of PI*2/3   from -PI/6 to PI/2
float PITHETA5=0;       //10 incriments to cover distance of PI       from -PI/2 to PI/2

void setup() {
  Serial.begin(115200);
  //MatrixMath();
  DistanceCalc();
}

void loop() {  

//import and update desired position 
// when it recieves new position Distance calc
// New Pxd
// New Pyd
// New Pzd
//if(new coordinates do not equal current coordinates){run Distance calc}

}

//------matrix math------// 
void MatrixMath(){

  //caclulate the homogeneous transformations for the movement in reference to the starting position
  //setting up the matricies  
  BLA::Matrix<4, 4> BASE =        {cos(theta0),0,-sin(theta0),0,0,1,0,0,sin(theta0),0,cos(theta0),0,0,0,0,1}; //rortate about the Y
  BLA::Matrix<4, 4> BASETR =      {1,0,0,0,0,1,0,a0,0,0,1,0,0,0,0,1};//Shift along the Y
  BLA::Matrix<4, 4> SHOULDER =    {cos(theta1),-sin(theta1),0,0,sin(theta1),cos(theta1),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
  BLA::Matrix<4, 4> SHOULDERTR =  {1,0,0,0,0,1,0,a1,0,0,1,0,0,0,0,1};//Shift along the Y
  BLA::Matrix<4, 4> ELBOW =       {cos(theta2),-sin(theta2),0,0,sin(theta2),cos(theta2),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
  BLA::Matrix<4, 4> ELBOWTR=      {1,0,0,0,0,1,0,a2,0,0,1,0,0,0,0,1};//Shift along the Y
  BLA::Matrix<4, 4> FORARM =      {cos(theta3),0,-sin(theta3),0,0,1,0,0,sin(theta3),0,cos(theta3),0,0,0,0,1}; //Rotate about the Y
  BLA::Matrix<4, 4> FORARMTR=     {1,0,0,0,0,1,0,a3,0,0,1,0,0,0,0,1};//Shift along the Y
  BLA::Matrix<4, 4> WRIST =       {cos(theta4),-sin(theta4),0,0,sin(theta4),cos(theta4),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
  BLA::Matrix<4, 4> WRISTTR=      {1,0,0,0,0,1,0,a4,0,0,1,0,0,0,0,1};//Shift along the Y
  //-----------------------------------------------------------------------------------------------------//
  
  //doing the matrix math   
  BLA::Matrix<4, 4> BASETOTAL =     BASE*BASETR;
  BLA::Matrix<4, 4> SHOULDERTOTAL = SHOULDER*SHOULDERTR;
  BLA::Matrix<4, 4> ELBOWTOTAL =    ELBOW*ELBOWTR;
  BLA::Matrix<4, 4> FORARMTOTAL =   FORARM*FORARMTR;
  BLA::Matrix<4, 4> WRISTTOTAL =    WRIST*WRISTTR;

  BLA::Matrix<4, 4> TOTAL=BASETOTAL*SHOULDERTOTAL*ELBOWTOTAL*FORARMTOTAL*WRISTTOTAL;
 
  
  //calculate the new position matrix
  BLA::Matrix<4> POSF = {TOTAL(0,3),TOTAL(1,3),TOTAL(2,3),0}; //what is calculated each time 

  //*need to change to inherit previous calc*

  //these are to display the various values useful for checking math
  //Serial << "POSF: " << POSF << '\n';       
  //Serial << "POSFX: " << TOTAL(0,3) << '\n';
  //Serial << "POSFY: " << TOTAL(1,3) << '\n';
  //Serial << "POSFZ: " << TOTAL(2,3) << '\n';
  //Serial << "POSFD: " << POSF(3) << '\n';

  //Serial << "BASETOTAL: " << BASETOTAL << '\n';
  //Serial << "SHOULDERTOTAL: " << SHOULDERTOTAL << '\n';
  //Serial << "ELBOWTOTAL: " << ELBOWTOTAL << '\n';
  //Serial << "FORARMTOTAL: " << FORARMTOTAL << '\n';
  //Serial << "WRISTTOTAL: " << WRISTTOTAL << '\n';

  Serial << "TOTAL: " << TOTAL << '\n';

  //------Distance Calculation------//
  distance=sqrt(sq(Pxd-TOTAL(0,3))+sq(Pyd-TOTAL(1,3))+sq(Pzd-TOTAL(2,3)));
  Serial.print("Distance: ");//displays th edistance calculation on each calculation 
  Serial.println(distance);

}

void DistanceCalc(){
 //MatrixMath();

 for(theta1=0;theta1>= -PI/2; theta1= theta1-PITHETA1){
   Serial.print("theta1: ");
   Serial.println(theta1);
   MatrixMath();
   Serial.println("---------- ");   
   if(distance<10){goto LocationReached;}
  
   for(theta2=0;theta2>= -PI/2; theta2= theta2-PITHETA2){
     Serial.print("theta2: ");
     Serial.println(theta2);
     MatrixMath();
     Serial.println("---------- ");   
     if(distance<10){goto LocationReached;}
      //logic for parallel adjustments for the gripper 
    }
   theta2=PITHETA2;
 }
  LocationReached:
  theta0=theta0*180/PI;
  theta1=theta1*180/PI;
  theta2=theta2*180/PI;
  theta3=theta3*180/PI;
  theta4=theta4*180/PI;

  Serial.print("theta0: ");
  Serial.println(theta0);
  Serial.print("theta1: ");
  Serial.println(theta1);
  Serial.print("theta2: ");
  Serial.println(theta2);
  Serial.print("theta3: ");
  Serial.println(theta3);
  Serial.print("theta4: ");
  Serial.println(theta4);

 //upload theta1,2,3 etc to the other board
}

