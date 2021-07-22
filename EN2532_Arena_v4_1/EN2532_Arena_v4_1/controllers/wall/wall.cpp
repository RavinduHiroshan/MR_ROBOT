


int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  
  int timeStep = (int)robot->getBasicTimeStep();
  

  Motor *wheels[2];
  char wheels_names[2][10] = {"l_wh","r_wh"};
  
  for (int i =0;i<2 ; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  
  }
  

  delete robot;
  return 0;
}
