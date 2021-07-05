
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define TIME_STEP 64
using namespace webots;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *base;
  base=robot->getMotor("base");
  double base_rotate=0.0 ;
  
  Motor *left;
  left=robot->getMotor("left");
  double left_rotate=0.0 ;
  
  Motor *right;
  right=robot->getMotor("right");
  double right_rotate=0.0 ;
  
  Motor *left_g;
  left_g=robot->getMotor("left_g");
  double left_g_rotate=0.0 ;
  
  Motor *right_g;
  right_g=robot->getMotor("right_g");
  double right_g_rotate=0.0 ;
  
  right->setPosition(-0.3);
  left->setPosition(-0.3);
  
  while (robot->step(TIME_STEP) != -1) {
  
    if (left_rotate<0.3){
    left_rotate+=0.005;
  }
  left->setPosition(left_rotate);

  
  if (right_rotate<0.3){
    right_rotate+=0.005;
  }
  right->setPosition(right_rotate);
  if ((left_rotate>0.1) && (right_rotate>0.1)){
  break;
  }
  }
  while (robot->step(TIME_STEP) != -1){
  if (base_rotate<1){
    base_rotate+=0.005;
  }
  base->setPosition(base_rotate);
  if (base_rotate>0.6){
    break;
  }
  }
  
  while (robot->step(TIME_STEP) != -1) {
  
    if (left_g_rotate<2){
    left_g_rotate+=0.01;
  }
  left_g->setPosition(left_g_rotate);

  
  if (right_g_rotate<2){
    right_g_rotate+=0.01;
  }
  right_g->setPosition(right_g_rotate);
  if ((left_g_rotate>1.57) && (right_g_rotate>1.57)){
  break;
  }
  }
  delete robot;
  return 0;  // EXIT_SUCCESS
}