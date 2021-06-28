// File:          lineFolowing.cpp
// Date:6/27/2020
// Description: Code for only line following
// Author: Group
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
#define MIN_GS 600 
#define NEW_GS 1000

// Overlap factor
#define OL 200

int lfm_speed[2];

long P=0, I=0, D=0, pErr=0, PID=0;
float Kp=0.80; // 0.80
float Ki=0.00; // 0.00
float Kd=0.02; // 0.02


#define LFM_FS 1000 //2000

// All the webots classes are defined in the "webots" namespace
using namespace webots;


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

void LineFollowingModule(void) {
  // Error Position Calculation & PID
  P = Position - NEW_GS*(NB_GROUND_SENS-1)/2;
  I = P + pErr;
  D = P - pErr;

  PID = Kp*P + Ki*I + Kd*D;
  
  pErr = P;
 
  lfm_speed[LEFT] = LFM_FS + PID;
  lfm_speed[RIGHT] = LFM_FS - PID;

}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  int speed[2];
  

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  
  // initialize devices
  for (int i = 0; i < 8; i++) {
    gs[i] = robot->getDistanceSensor(gsNames[i]);
    gs[i]->enable(TIME_STEP);
  }
  
  Motor *leftMotor = robot->getMotor("left_wheel");
  Motor *rightMotor = robot->getMotor("right_wheel");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    ReadGroudSensors();
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // read sensors outputs
    
    // Process sensor data here.
    ReadGroudSensors();
      
    // Speed initialization

    speed[LEFT] = 0;
    speed[RIGHT] = 0;

    // *** START OF SUBSUMPTION ARCHITECTURE ***

    // LFM - Line Following Module
    LineFollowingModule();
    speed[LEFT] = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];
    // Routines used when detecting the line
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
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    leftMotor->setVelocity(0.00628 * speed[LEFT]);
    rightMotor->setVelocity(0.00628 * speed[RIGHT]);
    std::cout <<  gsNew[0]<<"  "<<gsNew[1]<<"  "<<gsNew[2]<<" "<<gsNew[3]<<"  "<<gsNew[4]<<"  "<<gsNew[5]<<"  "<<gsNew[6]<<"  "<<gsNew[7]<<std::endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
