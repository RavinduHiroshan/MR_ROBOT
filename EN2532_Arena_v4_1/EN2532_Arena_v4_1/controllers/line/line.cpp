// File:          lineFolowing.cpp
// Date:6/27/2020
// Description: Code for only line following
// Author: Hiroshan
// Modifications: null

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 16
#define MAX_SPEED 6.28
#define LEFT 0
#define RIGHT 1
// 8 IR ground color sensors
#define NB_GROUND_SENS 8
// 520
#define MAX_GS 900 
// 110
#define MIN_GS 720 
#define NEW_GS 1020
// Overlap factor
#define OL 200

int lfm_speed[2];
long P=0, I=0, D=0, pErr=0, PID=0;
float Kp=0.80; // 0.80
float Ki=0.00; // 0.00
float Kd=0.02; // 0.02

#define LFM_FS 500 //2000
// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();
int timeStep = (int)robot->getBasicTimeStep();

// IR Ground Sensors
double gsValues[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
short gsNew[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
// Test array
unsigned short maxGS[NB_GROUND_SENS]= {500, 500, 500, 500, 500, 500, 500, 500};
unsigned short minGS[NB_GROUND_SENS]= {500, 500, 500, 500, 500, 500, 500, 500};
// Reading Ground Sensors Module
unsigned long Position = 0;
bool online = false;
DistanceSensor *gs[8];
  char gsNames[8][4] = {
    "gs0", "gs1", "gs2", "gs3",
    "gs4", "gs5", "gs6", "gs7"
  };

Motor *grip_motor_0;
Motor *grip_motor_1;
Motor *grip_motor_2;
Motor *left_grip;
Motor *right_grip;

//Arm Variable 
double Arm0  = 0.0;
double Arm1  = 1.6;
double Arm2  = 1.6;
double Arm3  = 0.0;
double leftGrip = 0.0;
double rightGrip = 0.0;


//Read sensors functions
void ReadGroudSensors(void){
  online = false;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
  for(int i=0; i<NB_GROUND_SENS; i++){
    gsValues[i] = gs[i]->getValue();
    //    Max & Min detection
    if(gsValues[i]<minGS[i]) minGS[i]=gsValues[i];
    if(gsValues[i]>maxGS[i]) maxGS[i]=gsValues[i];
    // linear Interpolation
    gsNew[i] = ((float)gsValues[i]-MIN_GS)/(MAX_GS-MIN_GS)*NEW_GS;

    // Limited values between 0 and 1000 (NEW_GS)
    if(gsNew[i]>NEW_GS) gsNew[i]=NEW_GS;
    if(gsNew[i]<0) gsNew[i]=0;
    
    if(gsNew[i]>200)online = true;
    if(gsNew[i]>50){
      // Average groud sensor value
      avgS += (unsigned long)gsNew[i]*(i*NEW_GS);
      // Sum ground sensor value
      sumS += gsNew[i];
    }
  }
  if(online)Position = avgS/sumS; // Position Calculation
  else if(Position < NEW_GS*(NB_GROUND_SENS-1)/2)Position = 0; // Left Sensor Memory Position
  else Position = NEW_GS*(NB_GROUND_SENS-1); // Right Sensor Memory Position

}

//Arm Moving Function
void ArmMoving(int move){
  //Motor *grip_motor_3= robot->getMotor("fourthArm");
  bool Done =false;
  if(move == 0 ){
  bool boxPosition = false;
  bool grip = false;
    while (robot->step(timeStep) != -1) {
      if(!Done){
        boxPosition =true;
        if(Arm0 <= 1.4){Arm0+=0.005; boxPosition =false;}
        if(Arm1 <= 2.2){Arm1+=0.005; boxPosition =false;}
        if(Arm2 >= 0.8){Arm2-=0.005; boxPosition =false;}
        grip_motor_0->setPosition(Arm0);   
        grip_motor_1->setPosition(Arm1);
        grip_motor_2->setPosition(Arm2);
        if(boxPosition){
          grip = true;
          if(leftGrip <= 0.2){leftGrip+=0.005; grip =false;}
          if(rightGrip <= 0.2){rightGrip+=0.005; grip =false;}
          left_grip->setPosition(leftGrip);
          right_grip->setPosition(rightGrip);
          if(grip)break;
        }
      }   
    }
  }else if(move == 1){
    bool boxPosition = false;
    while (robot->step(timeStep) != -1) {
      if(!Done){
        boxPosition =true;
        if(Arm0 > 0){Arm0-=0.02; boxPosition =false;}//0
        if(Arm1 > 1.3){Arm1-=0.005; boxPosition =false;}//1.6
        if(Arm2 < 1.5){Arm2+=0.005; boxPosition =false;}//1.6
        grip_motor_0->setPosition(Arm0);   
        grip_motor_1->setPosition(Arm1);
        grip_motor_2->setPosition(Arm2);
        if(boxPosition){
          left_grip->setPosition(0);
          right_grip->setPosition(0);
          break;
        }
      }   
    }
  }else if(move == 2){

  }
}


//Line Following function
void LineFollowingModule(void) {
  // Error Position Calculation & PID
  P = Position - NEW_GS*(NB_GROUND_SENS-1)/2;
  I = P + pErr;
  D = P - pErr;
  PID = Kp*P + Ki*I + Kd*D;
  pErr = P;
  lfm_speed[LEFT] = LFM_FS - PID;
  lfm_speed[RIGHT] = LFM_FS + PID;

}


int main(int argc, char **argv) {
  // create the Robot instance.
  int speed[2];
  // initialize devices
  for (int i = 0; i < 8; i++) {
    gs[i] = robot->getDistanceSensor(gsNames[i]);
    gs[i]->enable(TIME_STEP);
  }
  
  Motor *leftMotor = robot->getMotor("left_wheel");
  Motor *rightMotor = robot->getMotor("right_wheel");
  grip_motor_0 = robot->getMotor("mainArm");
  grip_motor_1= robot->getMotor("secondArm");
  grip_motor_2= robot->getMotor("thirdArm");
  left_grip = robot->getMotor("leftGrip");
  right_grip = robot->getMotor("rightrip");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  
  while (robot->step(timeStep) != -1) {
    ReadGroudSensors();
    speed[LEFT] = 0;
    speed[RIGHT] = 0;
    
    LineFollowingModule();
    speed[LEFT] = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];
    
    if(!online){
      if(P == -NEW_GS*(NB_GROUND_SENS-1)/2){
        speed[LEFT] = -LFM_FS;
        speed[RIGHT] = LFM_FS;
      }
      if(P == NEW_GS*(NB_GROUND_SENS-1)/2){
        speed[LEFT] = LFM_FS;
        speed[RIGHT] = -LFM_FS;
      }
    }
    leftMotor->setVelocity(0.00628 * speed[LEFT]);
    rightMotor->setVelocity(0.00628 * speed[RIGHT]);
    // leftMotor->setVelocity(5);
    // rightMotor->setVelocity(5);
    //std::cout <<  gsValues[0]<<"  "<<gsValues[1]<<"  "<<gsValues[2]<<" "<<gsValues[3]<<"  "<<gsValues[4]<<"  "<<gsValues[5]<<"  "<<gsValues[6]<<"  "<<gsValues[7]<<std::endl;
    std::cout <<  gsNew[0]<<"  "<<gsNew[1]<<"  "<<gsNew[2]<<" "<<gsNew[3]<<"  "<<gsNew[4]<<"  "<<gsNew[5]<<"  "<<gsNew[6]<<"  "<<gsNew[7]<<std::endl;
 };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}