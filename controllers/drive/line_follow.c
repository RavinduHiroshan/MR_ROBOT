/*
 * File:          Speed_Line_Follower_Robot_V4.c
 * Date:          October-04-2020
 * Description:   This is the controller to simulate the operation 
                  of the robot, similar to the physical model
 * Author:        DrakerDG (https://www.youtube.com/user/DrakerDG)
 * Modifications: V1
 */

#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define LEFT 0
#define RIGHT 1
#define TIME_STEP 32

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
#define NB_LEDS 5
#define MAX_GS 520 // 520
#define MIN_GS 110 // 110
#define NEW_GS 1000

// Overlap factor
#define OL 200

// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
short gs_new[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
// Test array
unsigned short maxGS[NB_GROUND_SENS]= {500, 500, 500, 500, 500, 500, 500, 500};
unsigned short minGS[NB_GROUND_SENS]= {500, 500, 500, 500, 500, 500, 500, 500};


// Motors
WbDeviceTag left_motor, right_motor;

// Reading Ground Sensors Module
unsigned long Position = 0;
bool online = false;

void ReadGroudSensors(void){
  online = false;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
    
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);

//    Max & Min detection
    if(gs_value[i]<minGS[i]) minGS[i]=gs_value[i];
    if(gs_value[i]>maxGS[i]) maxGS[i]=gs_value[i];
    
    // linear Interpolation
    gs_new[i] = ((float)gs_value[i]-MIN_GS)/(MAX_GS-MIN_GS)*NEW_GS;

    // Limited values between 0 and 1000 (NEW_GS)
    if(gs_new[i]>NEW_GS) gs_new[i]=NEW_GS;
    if(gs_new[i]<0) gs_new[i]=0;
    
    if(gs_new[i]>200)online = true;
    if(gs_new[i]>50){
      // Average groud sensor value
      avgS += (unsigned long)gs_new[i]*(i*NEW_GS);
      // Sum ground sensor value
      sumS += gs_new[i];
    }
  }
  if(online)Position = avgS/sumS; // Position Calculation
  else if(Position < NEW_GS*(NB_GROUND_SENS-1)/2)Position = 0; // Left Sensor Memory Position
  else Position = NEW_GS*(NB_GROUND_SENS-1); // Right Sensor Memory Position

}

int lfm_speed[2];

long P=0, I=0, D=0, pErr=0, PID=0;
float Kp=0.80; // 0.80
float Ki=0.00; // 0.00
float Kd=0.02; // 0.02


#define LFM_FS 3500 //2000

void LineFollowingModule(void) {
  // Error Position Calculation & PID
  P = Position - NEW_GS*(NB_GROUND_SENS-1)/2;
  I = P + pErr;
  D = P - pErr;

  PID = Kp*P + Ki*I + Kd*D;
  
  pErr = P;
  
//  printf("GS: %4d %4d %4d;   Max: %4d %4d %4d;   Min: %4d %4d %4d  \n", gs_value[0], gs_value[1], gs_value[2], maxGS[0], maxGS[1], maxGS[2], minGS[0], minGS[1], minGS[2]);

  lfm_speed[LEFT] = LFM_FS + PID;
  lfm_speed[RIGHT] = LFM_FS - PID;

}


/*
 * This is the main program.
 */
int main() {
  int i, speed[2];
  /* necessary to initialize webots stuff */
  wb_robot_init();


  /* initialization */
  char name[20];
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  for (;;) {
    // Run one simulation step
    wb_robot_step(TIME_STEP);

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

    printf("%4d   %4d   %4d   %4d   %4d   %4d   %4d   %4d   %4d   %5d   OnLine: %d  \n", gs_new[0], gs_new[1], gs_new[2], gs_new[3], gs_new[4], gs_new[5], gs_new[6], gs_new[7], (int)Position, (int)P, online);
  
    // Speed computation
    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
  };

  wb_robot_cleanup();

  return 0;
}
