/**
 * @file main.cpp
 * @authors Vivek Sood
 * @brief primary source file for testing
 * @date 2021-11-15
 * @copyright Copyright (c) 2021
 *
 * 
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}