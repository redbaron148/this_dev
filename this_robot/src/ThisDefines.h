#ifndef THIS_DEFINES_H
#define THIS_DEFINES_H

// Servo Indicies
#define BASE                0
#define SHOULDER            1
#define ELBOW               4
#define WRIST_FLEX          3
#define WRIST_TWIST         5
#define CLAW                2

#define OPEN                100
#define CLOSE               0

#define START_POSITION      60,10,0,0,10,0
#define REST_POSITION       0,10,gsp(CLAW),0,10,0
#define END_POSITION        0,10,gsp(CLAW),0,10,0
#define SCORE_POSITION_LEFT 90,110,gsp(CLAW),60,90,0
#define UP_POSITION         65,gsp(CLAW),40,100,0
#define OUT_POSITION        120,gsp(CLAW),60,110,0
#define SWIPE_MIDDLE        15,90,0,80,80,0
#define SWIPE_DOWN          30,120,100,125,45,0
#define SWIPE_START_POSITION -70,5,0,0,5,0

#define HAS_SOMETHING_ANGLE 140
#define TWIST_HAS_SOMETHING_ANGLE 45

#define SERVO_SPEED         0.20  //speed of slowest servo on arm (per 60 degrees)

// Info About the camera frames
#define IMAGE_WIDTH         320
#define IMAGE_HEIGHT        240
#define IMAGE_FOV           46
  
#define CENTER_X            IMAGE_WIDTH/2
#define CENTER_Y            IMAGE_HEIGHT/2

#define CENTER_X_OFFSET     20
#define CENTER_Y_OFFSET     20

#define MAX_POM_SIZE        1100
#define MAX_POM_WIDTH       40

#define ARM_CENTER_X        CENTER_X+40
#define ARM_CENTER_Y        CENTER_Y

#define MAX_ARM_REACH       22.5
#define MIN_ARM_REACH       10.

#ifndef PIE
#define PIE 3.14159
#endif

// Color channels
enum  CHANNEL{ 
  orange=0, 
  botguy, 
  yellow, 
  blue, 
  greenFoam, 
  greenPom,
  orangePom,
  orangeFoam
};

enum DIRECTION{
  left=0,
  right,
  forward,
  backward
};

// Strings for nice printing to screen, makes debuging flow easier 
std::string channel[8]  = {"orange",
                           "botguy",
                           "yellow pom",
                           "blue foam ball",
                           "green foam ball",
                           "green pom",
                           "orange foam ball",
                           "orange pom"};
                 
std::string servo[6]    = {"base",
                           "shoulder",
                           "claw",
                           "wrist flex",
                           "wrist twist"};

#endif
