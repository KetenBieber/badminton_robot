#include <ros/ros.h>

#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "helloworld");

  std::cout << "Hello World!" << std::endl;
  return 0;
}