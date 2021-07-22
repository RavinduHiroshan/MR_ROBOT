/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description: simple example of motor position control for Hinge2Joint
 */

#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag motor2 = wb_robot_get_device("motor 2");
 
  //consider the time step when creating the logic for gate
  int counter = 0;
  while (wb_robot_step(time_step) != -1) {
    counter++;
    
    if (counter == 50)
      wb_motor_set_position(motor2, -1.57); //1.57 radians -> 90 degree (- mark for direction)
    else if (counter == 100)
      wb_motor_set_position(motor2, 0);

    }
  

  wb_robot_cleanup();
  return 0;
}
