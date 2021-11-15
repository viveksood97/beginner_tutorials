/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    listener.cpp
 *  @author  Vivek Sood
 *  @date    11/15/2021
 *  @version 1.0
 *  @brief   Subscriber
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>

#include "beginner_tutorials/changeString.h"


/**
*   @brief A test to ensure that the change_string
* service is working properly
*/
TEST(Talker, testTalkerService) {
    ros::NodeHandle n;

    ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::changeString>("change_string");
    beginner_tutorials::changeString srv;
    srv.request.before = "Checking Service";

    client.call(srv);

    EXPECT_STREQ(srv.response.after, "Checking Service");
}
