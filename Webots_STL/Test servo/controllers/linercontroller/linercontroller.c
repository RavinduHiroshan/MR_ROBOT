#include <math.h>
#include <webots/motor.h>
#include <webots/robot.h>

int main(int argc, char **argv) {
  wb_robot_init();
  double time_step = wb_robot_get_basic_time_step();
  WbDeviceTag motor = wb_robot_get_device("linear motor");
  double target = 0;
  int counter = 0;
  /* This simple algorithm moves the linear motor to position 0   */
  /* and then progresses by steps of 2 cm until it reaches 20 cm. */
  /* Once done, it comes back to position 0 and restarts.         */
  /* The duration of each step is 100 * time_step = 1.6 second    */
  /* (assuming time_step = 16 milliseconds)                       */
  while (wb_robot_step(time_step) != -1) {
    wb_motor_set_position(motor, target);
    if (counter++ == 100) { /* 1.6 seconds elapsed */
      target += 0.01;       /* 2 cm */
      if (target > 0.05)     /* 20 cm */
        target = 0;
      counter = 0;
    }
  };
  wb_robot_cleanup();
  return 0;
}
