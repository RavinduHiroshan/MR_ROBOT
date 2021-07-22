// File:          lineFolowing.cpp
// Date:6/27/2020
// Description: Code for only line following
// Author: Hiroshan
// Modifications: null
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/positionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 16
#define MAX_SPEED 6.28

//motor let and right
#define LEFT 0
#define RIGHT 1

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
// maximu reading of ir array
#define MAX_GS 900 
// 110// maximu reading of ir array
#define MIN_GS 720 

#define NEW_GS 1020
// Overlap factor
#define OL 200

int lfm_speed[2];
  int speed[2];
long P=0, I=0, D=0, pErr=0, PID=0;

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

//Drive 
Motor *leftMotor;
Motor *rightMotor;
//Encoders
PositionSensor *left_sensor;
PositionSensor *right_sensor;
//distance
DistanceSensor *ds[4];
char dsNames[5][10] = {"ds_left","ds_right","ds_left1","ds_right1"};



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
    
    if(gsNew[i]<200)online = true;
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
int ArmMoving(int move){
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
   return 0;
}


//Line Following function
void LineFollowingModule(void) {
  float Kp=0.80; // 0.80
  float Ki=0.00; // 0.00
  float Kd=0.02; // 0.02
  // Error Position Calculation & PID
  P = Position - NEW_GS*(NB_GROUND_SENS-1)/2;
  I = P + pErr;
  D = P - pErr;
  PID = Kp*P + Ki*I + Kd*D;
  pErr = P;
  lfm_speed[LEFT] = LFM_FS - PID;
  lfm_speed[RIGHT] = LFM_FS + PID;

}

int turning(int direction){
  left_sensor->enable(timeStep);
  right_sensor->enable(timeStep);
  float K=0.1;
  double angle=90;
  double count=0;
  double wheelRadius=3;
  double pivotDiam=23.72;
  double pivotCirc= 3.14* pivotDiam;
  double targetDistance = (abs(angle) / 360.0) * pivotCirc;
  double targetCount= targetDistance / wheelRadius;
  if(direction == 0 ){
    leftMotor->setVelocity((-K) * MAX_SPEED);
    rightMotor->setVelocity((K) * MAX_SPEED);
    while (robot->step(timeStep) != -1){
      if (count<targetCount){
        count = right_sensor->getValue();
        //std::cout<<count<<" "<<targetCount<<std::endl;
      }
      if (count>targetCount){
        //std::cout<<"Break"<<std::endl;
        leftMotor->setVelocity((0) * MAX_SPEED);
        rightMotor->setVelocity((0) * MAX_SPEED);
        break;
      } 
    }
  
 }else if(direction == 1 ){
    leftMotor->setVelocity((K) * MAX_SPEED);
    rightMotor->setVelocity((-K) * MAX_SPEED);
    while (robot->step(timeStep) != -1){
      if (count<targetCount){
        count = left_sensor->getValue();
        //std::cout<<count<<" "<<targetCount<<std::endl;
      }
      if (count>targetCount){
        //std::cout<<"Break"<<std::endl;
        leftMotor->setVelocity((0) * MAX_SPEED);
        rightMotor->setVelocity((0) * MAX_SPEED);
        break;
      } 
    }
  
 }
 return 0;
};
  
int LineFoloowingExe(){
    ReadGroudSensors();
    speed[LEFT] = 0;
    speed[RIGHT] = 0;
    LineFollowingModule();
    speed[LEFT] = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];
    leftMotor->setVelocity(0.00628 * speed[LEFT]);
    rightMotor->setVelocity(0.00628 * speed[RIGHT]);
    // leftMotor->setVelocity(5);
    // rightMotor->setVelocity(5);
    //std::cout <<  gsValues[0]<<"  "<<gsValues[1]<<"  "<<gsValues[2]<<" "<<gsValues[3]<<"  "<<gsValues[4]<<"  "<<gsValues[5]<<"  "<<gsValues[6]<<"  "<<gsValues[7]<<std::endl;
    std::cout <<online<<"  "<<gsNew[1]<<"  "<<gsNew[2]<<" "<<gsNew[3]<<"  "<<gsNew[4]<<"  "<<gsNew[5]<<"  "<<gsNew[6]<<"  "<<gsNew[7]<<std::endl;
    return 0;
};

int wallFollowing(){
  double Kp = 0.4;
  double Kd = 0.05;
  double maxSpeed = 5;
  double lastError = 0.0;
  double transitLastError= 0.0;
  double wallDetectThreshHold =25.0;
  double followingDistance =12.0;
  double k1 =0.7;
  double k2 = 0.5;
  double k3 = 0.2;
  double k4 = 0.6;
  double k5 = 0.2;

  while (robot->step(timeStep) != -1) {
    //  double frontVal = ds[2]->getValue();
    double leftVal = ds[0]->getValue();
    double rightVal = ds[1]->getValue();
    double leftVal1 = ds[2]->getValue();
    double rightVal1 = ds[3]->getValue();
    
    double rightIr=(0.1594*pow(rightVal,-0.8533)-0.02916)*100;
    double rightIr1=(0.1594*pow(rightVal1,-0.8533)-0.02916)*100;
    double leftIr=(0.1594*pow(leftVal,-0.8533)-0.02916)*100;
    double leftIr1=(0.1594*pow(leftVal1,-0.8533)-0.02916)*100;
    
    if(rightIr< wallDetectThreshHold || leftIr<wallDetectThreshHold){
      if((rightIr<wallDetectThreshHold && rightIr1<wallDetectThreshHold) && leftIr>wallDetectThreshHold){
        double readDistance = (rightIr+rightIr1)*0.5;
        double differenceError = rightIr-rightIr1; 
        double distanceError = readDistance-followingDistance;
        double overallError = k1*differenceError + k2*distanceError;
        double adjust = (Kp*overallError + Kd*(overallError-lastError));
        lastError = overallError;
        leftMotor->setVelocity(0.5*(maxSpeed+adjust));
        rightMotor->setVelocity(0.5*(maxSpeed-adjust));
        std ::cout<<"differenceError  "<<differenceError<<"  distanceError  "<<distanceError<<"  overallError  "<< overallError <<"  adjust "<<adjust<<std::endl;
        
      
      }else if((leftIr<wallDetectThreshHold && leftIr1<wallDetectThreshHold) && rightIr>wallDetectThreshHold){
        double readDistance = (leftIr+leftIr1)*0.5;
        double differenceError = leftIr-leftIr1; 
        double distanceError = readDistance-followingDistance;
        double overallError = k4*differenceError + k5*distanceError;
        
       // lastError -= overallError;
        double adjust = (Kp*overallError + Kd*(overallError-lastError));
        lastError = overallError;
        leftMotor->setVelocity(0.5*(maxSpeed-adjust));
        rightMotor->setVelocity(0.5*(maxSpeed+adjust));
        std ::cout<<"differenceError  "<<differenceError<<"  distanceError  "<<distanceError<<"  overallError  "<< overallError <<"  adjust "<<adjust<<std::endl;
        
      }else if((rightIr<wallDetectThreshHold && rightIr1>wallDetectThreshHold) || (leftIr<wallDetectThreshHold && leftIr1>wallDetectThreshHold)){
        if(rightIr<followingDistance){
          double error = rightIr-followingDistance;
          double adjust = k3*(error);
          leftMotor->setVelocity(0.5*(maxSpeed+adjust));
          rightMotor->setVelocity(0.5*(maxSpeed-adjust));
          std ::cout<<"start  "<<adjust<<std::endl;
         
        }else{
          double error = rightIr-followingDistance;
          double adjust = k3*(error);
          leftMotor->setVelocity(0.5*(maxSpeed+adjust));
          rightMotor->setVelocity(0.5*(maxSpeed-adjust));  
          std ::cout<<"start  "<<adjust<<std::endl;
        }
      
      }else if(rightIr<wallDetectThreshHold && leftIr<wallDetectThreshHold){
        
        //double gap=(rightIr+leftIr)*0.5;
        double differenceError = rightIr-rightIr1;
        double error = k1*(rightIr-leftIr) + k2*differenceError;
        
        double adjust =(Kp*error + Kd*(error-transitLastError))*0.5;
        transitLastError=error;
        
        leftMotor->setVelocity(0.5*(maxSpeed-adjust));
        rightMotor->setVelocity(0.5*(maxSpeed+adjust));   
        std ::cout<<"adjust  "<<adjust<< "  differenceError    "  << differenceError<<std::endl;
                         
      } 
        }
    
    }
return 0;
};



int main(int argc, char **argv) {
  // initialize devices
  for (int i = 0; i < 8; i++) {
    gs[i] = robot->getDistanceSensor(gsNames[i]);
    gs[i]->enable(TIME_STEP);
  }
  for (int i=0; i<4; i++){
  ds[i] = robot->getDistanceSensor(dsNames[i]);
  ds[i]->enable(TIME_STEP);
  }
  //Drive 
  leftMotor = robot->getMotor("left_wheel");
  rightMotor = robot->getMotor("right_wheel");
  //Encoders
  left_sensor = robot->getPositionSensor("ps_left");
  right_sensor = robot->getPositionSensor("ps_right");
  
  //Arm motors
  grip_motor_0 = robot->getMotor("mainArm");
  grip_motor_1= robot->getMotor("secondArm");
  grip_motor_2= robot->getMotor("thirdArm");
  left_grip = robot->getMotor("leftGrip");
  right_grip = robot->getMotor("rightrip");
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  
  while (robot->step(timeStep) != -1) {
    LineFoloowingExe(); //    
    
 };

  // Enter here exit cleanup code.
  delete robot;
  return 0;
}