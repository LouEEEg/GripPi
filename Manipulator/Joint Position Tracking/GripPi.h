/*
*   GripPi.h  
*   CSUS EEE193 Senior Product Design - Team GripPi
*   
*   File description: Header file includes definitions and structures 
*                     for the control of GripPi manipulator
*/

// ----- Stepper Drivers ----- //
// --------------------------- //
#define DRIVER_INTERFACE        1      // Stepper driver with PUL & DIR pins
 
#define BASE_MAX_SPEED          2000
#define BASE_RUN_SPEED          500
#define BASE_ACCELERATION       300

#define SHOULDER_MAX_SPEED      1600
#define SHOULDER_RUN_SPEED      500
#define SHOULDER_ACCELERATION   200

#define ELBOW_MAX_SPEED         12000
#define ELBOW_RUN_SPEED         800
#define ELBOW_ACCELERATION      1800

#define FOREARM_MAX_SPEED       2000
#define FOREARM_RUN_SPEED       500
#define FOREARM_ACCELERATION    200

#define WRIST_MAX_SPEED         2000
#define WRIST_RUN_SPEED         500
#define WRIST_ACCELERATION      300

// ----- Joint Pin Assignment ----- //
// -------------------------------- //
// Driver #1                        // All drivers use the following pin/color combination
#define BASE_LIMIT          10      // Digital/in  GREEN
#define BASE_ENA            11      // Dgitial/out WHITE
#define BASE_PUL            12      // Digital/out ORANGE
#define BASE_DIR            13      // PWM         YELLOW

// Drivers #2 & #3
#define SHOULDER_LIMIT      52
#define SHOULDER_ENA        4
#define SHOULDER_PUL        5
#define SHOULDER_DIR        6

// Driver #4
#define ELBOW_LIMIT         51
#define ELBOW_ENA           32
#define ELBOW_PUL           33
#define ELBOW_DIR           34

// Driver #5
#define FOREARM_LIMIT       50
#define FOREARM_ENA         9
#define FOREARM_PUL         8
#define FOREARM_DIR         7

// Driver #6
#define WRIST_LIMIT         49
#define WRIST_ENA           47
#define WRIST_PUL           46
#define WRIST_DIR           45

// ----- Serial Due/Raspberry Pi ----- //
// ----------------------------------- //
#define SERIAL_BUFFER_SIZE 32


// ----- Manipulator Structure ----- //
// --------------------------------- //
typedef struct{
  struct{
    // Length: Joint 2 Joint, in mm
    float base_2_shoulder = 221;
    float shoulder_2_elbow = 221; 
    float elbow_2_wrist = 218; 
    float wrist_2_gripper = 147;
  }length;

  struct {
    // Angle of each joint in RAD
    float base = 0;
    float shoulder = 0;
    float elbow = 0;
    float wrist = 0;
  }angle;

  struct {
    // Number of steps to run 
    int base = 0;
    int shoulder = 0;
    int elbow = 0;
    int forearm = 0;
    int wrist = 0;
    int gripper = 0;
  }steps;

  struct{
    // Position of each angle 
    long base = 0;
    long shoulder = 0;
    long elbow = 0;
    long forearm = 0;
    long wrist = 0;
  }position;

  struct{
    // Goal Position, distance is reach of the arm
    float distance = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float g = 0;
  }goal;

  struct{
    float x = 0;
    float y = 0;
    float z = 0;
    float g = 0; 
  }pose;
}Manipulator;

typedef struct{
  char receivedChars[SERIAL_BUFFER_SIZE];
  char tempChars[SERIAL_BUFFER_SIZE];
  float Rx_x = 0.0;
  float Rx_y = 0.0;
  float Rx_z = 0.0;
  float Rx_g = 0.0;
  bool new_data = false;
}SerialInput;
