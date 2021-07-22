
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 16
#define c 800


#define TIME_STEP 16
#define MAX_SPEED 6.28


int flag=1;
////////////wall following//////////////////////////////////

long speed[2];
long p=0, i=0, d=0, pid=0;

float kP=0.02; // 0.80
float kI=0.002; // 0.00
float kD=0.001; // 0.02

long pr_error=0;
long total_er=0;
long r_error;
 
long pl_error=0;
long total_el=0;
long l_error;

/////////////////////////////////////////////////

////////////////////////////////////////////////////////
using namespace webots;
Robot *robot = new Robot();

////////////////////turn function/////////////////////////
void turn(){ 


float radius=0.032; //m
float encoder_unit=(2*3.14*radius)/6.28; //  m/rad
float wheel_distance=0.14 ;
  
double ps_values[2]={0.,0.}; //0-left,1-right
double last_ps_values[2]={0.,0.};
double distance[2]={0.,0.};  //0-left,1-right
double diff=0;
double w=0;
float angle=0;
int i=1;
double ps0=0.;
double ps1=0.;




  
  Motor *leftMotor = robot->getMotor("left_wheel");
  Motor *rightMotor = robot->getMotor("right_wheel");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  

  int timeStep = (int)robot->getBasicTimeStep();
  
  PositionSensor *left_ps=robot->getPositionSensor("psl");
  PositionSensor *right_ps=robot->getPositionSensor("psr");
  left_ps->enable(timeStep);
  right_ps->enable(timeStep);
  
  while (robot->step(timeStep) != -1 ) {
    
    
    ps_values[0]=left_ps->getValue();
    ps_values[1]=right_ps->getValue();
    if(i==1){
    ps0=ps_values[0];
    ps1=ps_values[1];
    i=2;
    }
    
    ps_values[0]-=ps0;
    ps_values[1]-=ps1;
    
    leftMotor->setVelocity(-2.0);
    rightMotor->setVelocity(2.0);
    
    for (int i = 0; i < 2 ; i++) {
      //diff=ps_values[i]-last_ps_values[i];
     diff=last_ps_values[i]-ps_values[i];
      distance[i]=diff*encoder_unit; //distance in m for this instance
      
    }
    
    w=(distance[0]-distance[1])/wheel_distance ;//angle in clocwise direction

    angle+=w;
    std::cout<<"angle"<<angle<<std::endl;
    std::cout<<"---------------------"<<std::endl;
     for (int i = 0; i < 2 ; i++) {
       last_ps_values[i]=ps_values[i];
       
     }
    if (angle>1.7){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    break;
    }
    
  };





}

/////////////////////////////////////////////////////////

/////////////motor run function/////////////////////////

void motor_run(int m1,int m2){
    if (m1>10){
        m1=10;
        }
    if (m2>10){
        m2=10;
        }
    if (m1<0){
        m1=2;
        }
    if (m2<0){
        m2=2;
        }
    speed[0]=m1; //right speed
    speed[1]=m2; //left speed
    //std::cout<<"right:"<<m1<<"left"<<m2<<std::endl;
    
}

////////////////////////////////////////////////////////

////////////////////////////////////////////////////////

//////////////////////wall follow function///////////////////

/*
void wall_follow(){
  
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right","ds_left"};
  
 for (int i = 0; i < 2 ; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    }
   
  Motor *leftMotor = robot->getMotor("left_wheel");
  Motor *rightMotor = robot->getMotor("right_wheel");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  int timeStep = (int)robot->getBasicTimeStep();
  while (robot->step(timeStep) != -1) {
    int r_value=ds[0]->getValue();
    int l_value=ds[1]->getValue();
    
   
    if (r_value==1000 and l_value==1000){
        leftMotor->setVelocity(10);
        rightMotor->setVelocity(10); 
        }
    else if (r_value<1000){
        r_error=r_value-c;
        
        p=r_error*kP;
        d=(r_error-pr_error)*kD;
        i=total_er*kI;
        
        pid=p+d+i;
        
        pr_error=r_error;
        total_er+=r_error;
        
        
        motor_run(5-pid,5+pid);
        leftMotor->setVelocity(speed[1]);
        rightMotor->setVelocity(speed[0]);
        
        }
    else{
        l_error=l_value-c;
        
        p=l_error*kP;
        d=(l_error-pl_error)*kD;
        i=total_el*kI;
        
        pid=p+d+i;
        
        pl_error=l_error;
        total_el+=l_error;
        
        
        motor_run(5+pid,5-pid);
        leftMotor->setVelocity(speed[1]);
        rightMotor->setVelocity(speed[0]);

        }
    
     
  };



}*/
///////////////////////////////////////////////////////////////




////////////declaration of variable line following////////////
double sensorValues[12];
double initialSpeed = 4.00;
double set = 3500;
double e = 0;
double leftSpeed = 0;
double rightSpeed = 0;

// Convert to binary values
void convert(){
 for (int i = 0; i <10; i++){
   if (sensorValues[i] > 900){
     sensorValues[i] = 1;
   }else{
     sensorValues[i] = 0;
   }
 }

}


//////////////////////////////PID control/////////////////////
double PID_calc(){
  double avg = 0;
  double total = 0;
  for (int i = 0; i < 8 ; i++){ 
   avg += sensorValues[i] * i * 1000;
   total += sensorValues[i]; 
  }

 double position = avg / total;  //weighted mean
 
 double kp = 0.006;
 double kd = 0.00001;
 double ki = 0.00000000000001;
 double error = position - set;
 double P = kp * error;
 double D = kd * (error - e);
 double I = ki * (error + e);
 double offset = (P + D + I);
 e = error;
 
 return offset;
}


// To control the speed
double MSpeed(double speed){

 if (speed > 0){
   if (speed > 10){
     speed = 10;
   }
 }else{
   if (speed < -10){
     speed = -10;
   }
 }
 
 return speed;
}


int main(int argc, char **argv) {

  Motor *base;
  base=robot->getMotor("base");
  double base_rotate=0.0 ;
  
  base->setPosition(1);
  
// Initialize & enable the distance sensors  
  DistanceSensor *ir[12];
  char sensorNames[12][15] = {"gs0","gs1","gs2","gs3","gs4","gs5","gs6","gs7","left3","right3","sharp","sharp2"};
  for (int i = 0; i < 12; i++) {
    ir[i] = robot->getDistanceSensor(sensorNames[i]);
    ir[i]->enable(TIME_STEP);
  }
    
//Initialize the Motors 
  Motor *wheels[2];
  char wheels_names[2][15] = {"left_wheel","right_wheel"};
  for (int i = 0; i <2; i++){
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  
  
  
  

  
//Line following loop
  while (robot->step(TIME_STEP) != -1){

// To collect the sensor output
    for (int i = 0; i < 12 ; i++){
      sensorValues[i] = ir[i]->getValue();
    }
   
    
    convert(); //Covert sensor values to binary
    //Left sidejunction
    if (sensorValues[9]==1 && sensorValues[8]==0){
      wheels[0] ->setVelocity(0.0); 
      wheels[1] ->setVelocity(10.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }else{
        wheels[0] ->setVelocity(0.0); 
        wheels[1] ->setVelocity(10.0);
      }
    }
    //Right side junction
    else if(sensorValues[9]==0 && sensorValues[8]==1){
      wheels[0] ->setVelocity(10.0); 
      wheels[1] ->setVelocity(0.0);
      if (sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }else{
        wheels[0] ->setVelocity(10.0); 
        wheels[1] ->setVelocity(0.0);
      }
    }
    else if(sensorValues[8]==0 && sensorValues[9]==0){
      // At T junction turn right     
       wheels[0] ->setVelocity(8.0); 
       wheels[1] ->setVelocity(0.0);
      
     } // At 4 way crossing move forward
     else if(sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 && sensorValues[4]==0 && sensorValues[5]==0 && sensorValues[6]==0 && sensorValues[7]==0){
        wheels[0] ->setVelocity(4.0); 
        wheels[1] ->setVelocity(4.0);
        double offset = PID_calc();
        double leftSpeed =  initialSpeed-offset;
        double rightSpeed = initialSpeed+offset;
    
        double lefSpeed = MSpeed(leftSpeed) ;
        double rigSpeed = MSpeed(rightSpeed) ;
         
        wheels[0] ->setVelocity(lefSpeed); 
        wheels[1] ->setVelocity(rigSpeed);
      }
      
      
         
   
    
    else{
      double offset = PID_calc();
      double leftSpeed =  initialSpeed-offset;
      double rightSpeed = initialSpeed+offset;
    
      double lefSpeed = MSpeed(leftSpeed) ;
      double rigSpeed = MSpeed(rightSpeed) ;
         
      wheels[0] ->setVelocity(lefSpeed); 
      wheels[1] ->setVelocity(rigSpeed);
   
    }
    
  DistanceSensor *ds[2];
  char dsNames[2][10] = {"ds_right","ds_left"};
  
 for (int i = 0; i < 2 ; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
    }
    
    int r_value=ds[0]->getValue();
    int l_value=ds[1]->getValue();
    
    if(r_value<1000 || l_value<1000  ){
    
    while (robot->step(TIME_STEP) != -1) {
    int r_value=ds[0]->getValue();
    int l_value=ds[1]->getValue();
    
   
    if (r_value==1000 and l_value==1000){
        break;
        }
    else if (r_value<1000){
        r_error=r_value-c;
        
        p=r_error*kP;
        d=(r_error-pr_error)*kD;
        i=total_er*kI;
        
        pid=p+d+i;
        
        pr_error=r_error;
        total_er+=r_error;
        
        
        motor_run(5-pid,5+pid);
        wheels[0]->setVelocity(speed[1]);
        wheels[1]->setVelocity(speed[0]);
        
        }
    else{
        l_error=l_value-c;
        
        p=l_error*kP;
        d=(l_error-pl_error)*kD;
        i=total_el*kI;
        
        pid=p+d+i;
        
        pl_error=l_error;
        total_el+=l_error;
        
        
        motor_run(5+pid,5-pid);
        wheels[0]->setVelocity(speed[1]);
        wheels[1]->setVelocity(speed[0]);

        }
    
     
  };
        
    }
    
std::cout<<"shaaaaarrppppppppppp"<<sensorValues[10]<<std::endl;
std::cout<<"---------------------------------"<<std::endl;   
if(sensorValues[10]>0.55 && sensorValues[0]==0 && sensorValues[1]==0 && sensorValues[2]==0 && sensorValues[3]==0 ){
    std::cout<<"iiiiiiiiiiiiiiiiiiiiiiiiiiiii"<<std::endl;
    
   //for (int i = 0; i <6; i++){
    // wheels[0] ->setVelocity(6); 
    // wheels[1] ->setVelocity(6);
    // }
    
    turn();
    flag=0;
}  

if(flag==0 && sensorValues[11]>2.7){
    turn();
    turn();
    wheels[0] ->setVelocity(0.); 
    wheels[1] ->setVelocity(0.);
  
}    
    
//Print the binary output of sensors
    std::cout<<"df_left4 = "<<sensorValues[0]<<"  ";
    std::cout<<"df_left3 = "<<sensorValues[1]<<"  ";
    std::cout<<"df_left2 = "<<sensorValues[2]<<"  ";
    std::cout<<"df_left1 = "<<sensorValues[3]<<"  ";
    std::cout<<"df_right1 = "<<sensorValues[4]<<"  ";
    std::cout<<"df_right2 = "<<sensorValues[5]<<"  ";
    std::cout<<"df_right3 = "<<sensorValues[6]<<"  ";
    std::cout<<"df_right4 = "<<sensorValues[7]<<"  ";
    std::cout<<"df_left_most = "<<sensorValues[8]<<"  ";
    std::cout<<"df_right_most = "<<sensorValues[9]<<std::endl; 
   
  };
  
  delete robot;
  return 0;
  
  }
  
  