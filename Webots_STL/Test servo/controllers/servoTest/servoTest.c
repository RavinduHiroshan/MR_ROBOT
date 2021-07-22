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
 * Description: simple example of motor position control
 */

#include <math.h>
#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag motor = wb_robot_get_device("motor");

  double target = 0;
  int counter = 0;
  while (wb_robot_step(time_step) != -1) {
    wb_motor_set_position(motor, M_PI);
    // counter++;
    // if (counter == 50) {
      // target = M_PI_4;
      // //counter = 0;
    // }
    
  };
  wb_robot_cleanup();
  return 0;
}
