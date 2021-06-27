#include <webots/Robot.hpp>
#include <iostream>

using namespace webots;

int main() {
  Robot *robot = new Robot();

  while (robot->step(32) != -1)
    std::cout << "Hello World!" << std::endl;

  delete robot;
  return 0;
}