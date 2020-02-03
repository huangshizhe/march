// Copyright 2018 Project March.

#include "ros/ros.h"
#include "gtest/gtest.h"

/**
 * The main method which runs all the tests
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "testnode");
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
