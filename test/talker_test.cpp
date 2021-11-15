#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_msgs/String.h>

#include "beginner_tutorials/changeString.h"

TEST(Talker, testTalkerService) {
    ros::NodeHandle n;

    ros::ServiceClient client =
    n.serviceClient<beginner_tutorials::changeString>("change_string");
    beginner_tutorials::changeString srv;
    srv.request.before = "Check!!";

    client.call(srv);

    EXPECT_STREQ(srv.response.after, "Check!!");
}