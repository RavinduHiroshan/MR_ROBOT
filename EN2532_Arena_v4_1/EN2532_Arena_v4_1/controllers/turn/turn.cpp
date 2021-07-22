 int main(int argc, char **argv) {
  
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();
  double angle=180;
  
  double count=0;
  Motor *leftMotor = robot->getMotor("left_wheel");
  Motor *rightMotor = robot->getMotor("right_wheel");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  PositionSensor *left_sensor = robot->getPositionSensor("ps_left");
  PositionSensor *right_sensor = robot->getPositionSensor("ps_right");
  left_sensor->enable(timeStep);
  right_sensor->enable(timeStep);


  double wheelRadius=3;
  double pivotDiam=24.55;
  double pivotCirc= 3.14* pivotDiam;
  double targetDistance = (abs(angle) / 360.0) * pivotCirc;
  double targetCount= targetDistance / wheelRadius;
  
  leftMotor->setVelocity((-0.1) * MAX_SPEED);
  rightMotor->setVelocity((0.1) * MAX_SPEED);
  
  while (robot->step(timeStep) != -1) {
    while (robot->step(timeStep) != -1){
        if (count<targetCount){
      count = right_sensor->getValue();
      std::cout<<count<<" "<<targetCount<<std::endl;
    }
    if (count>targetCount){
      std::cout<<"Break"<<std::endl;
      leftMotor->setVelocity((0) * MAX_SPEED);
      rightMotor->setVelocity((0) * MAX_SPEED);
      break;
    } 
  
    }
   

    };

 
  delete robot;
  return 0;
}