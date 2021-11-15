/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    main.cpp
 *  @author  Vivek Sood
 *  @date    11/15/2021
 *  @version 1.0
 *  @brief   Main file to run all tests
 */


#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}
