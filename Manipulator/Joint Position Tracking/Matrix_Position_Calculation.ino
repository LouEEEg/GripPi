#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
 

#define PI 3.1415926535897932384626433832795 
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
float Pxd = 356; //example bin2 location 14inch = 355.6mm
float Pyd = 305; //example bin2 location 12inch = 304.8mm
float Pzd = 0; 

float distance = 0; 
float APx = 0;
float APy = 0;

void setup() {
  Serial.begin(115200);
  ThetaCalc();
  MatrixMath();
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
  BLA::Matrix<4, 4> WRIST =       {cos(theta3),-sin(theta3),0,0,sin(theta3),cos(theta3),0,0,0,0,1,0,0,0,0,1};//Rotate about the Z
  BLA::Matrix<4, 4> WRISTTR=      {1,0,0,0,0,1,0,a3,0,0,1,0,0,0,0,1};//Shift along the Y

  //-----------------------------------------------------------------------------------------------------//
  
  //doing the matrix math   
  BLA::Matrix<4, 4> BASETOTAL =     BASE*BASETR;
  BLA::Matrix<4, 4> SHOULDERTOTAL = SHOULDER*SHOULDERTR;
  BLA::Matrix<4, 4> ELBOWTOTAL =    ELBOW*ELBOWTR;
  BLA::Matrix<4, 4> WRISTTOTAL =    WRIST*WRISTTR;

  BLA::Matrix<4, 4> TOTAL=BASETOTAL*SHOULDERTOTAL*ELBOWTOTAL*WRISTTOTAL;
 
  //calculate the new position matrix
  BLA::Matrix<4> POSF = {TOTAL(0,3),TOTAL(1,3),TOTAL(2,3),0}; //what is calculated each time 

  //*need to change to inherit previous calc*

  //these are to display the various values useful for checking math
  //Serial << "POSF: " << POSF << '\n';       
  Serial << "POSFX: " << TOTAL(0,3) << '\n';
  Serial << "POSFY: " << TOTAL(1,3) << '\n';
  Serial << "POSFZ: " << TOTAL(2,3) << '\n';

  //Serial << "BASETOTAL: " << BASETOTAL << '\n';
  //Serial << "SHOULDERTOTAL: " << SHOULDERTOTAL << '\n';
  //Serial << "ELBOWTOTAL: " << ELBOWTOTAL << '\n';
  //Serial << "WRISTTOTAL: " << WRISTTOTAL << '\n';

  Serial << "TOTAL: " << TOTAL << '\n';
}

void ThetaCalc(){
//------calculating theta 0, theta 2, and theta3------//
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

}
