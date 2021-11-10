/**
 *  @copyright MIT License, © 2021 Vivek Sood
 *  @file    talker.cpp
 *  @author  Vivek Sood
 *  @date    11/08/2021
 *  @version 1.0
 *  @brief   Publisher
 *  @section DESCRIPTION
 *  A Publisher node that publishes a string.
 *  @mainpage This is beginners tutorial for creating a simple ROS package.
 */

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"


// To avoid cpplint issues
struct StringContainer {
  std::string currentString;
} container;

/**
 * @brief Service function to change the current string
 * @param request Request parameter
 * @param response Response parameter
 * @return true
 */
bool changeCurrentString(
  beginner_tutorials::changeString::Request &request,
  beginner_tutorials::changeString::Response &response) {
  response.after = request.before;
  container.currentString = response.after;
  return true;
}

/**
 * @brief Main function
 * @param argc number of input arguments
 * @param argv char pointer containing arguments
 * @return 0
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  container.currentString = argv[1];
  ros::init(argc, argv, "talker");

  if (!ros::ok()) {
    ROS_FATAL_STREAM("ros::ok false");
    return 1;
  }

    /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    ros::ServiceServer server =
        n.advertiseService("change_string", changeCurrentString);

    /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
    int count = 0;

    while (ros::ok()) {
      /**
     * This is a message object. You stuff it with data, and then publish it.
     */
      std_msgs::String msg;

      if (container.currentString == "DEFAULT") {
        ROS_WARN_STREAM
        ("The default string \"DEFAULT\" is still unchanged, use service");
      }

      std::stringstream ss;
      ss << container.currentString << " [id: " << count << "]";
      msg.data = ss.str();
      if (container.currentString.size() == 0) {
        ROS_ERROR_STREAM("Empty message not allowed");
      } else {
        ROS_DEBUG_STREAM("Updated String: " << container.currentString);
        chatter_pub.publish(msg);
        ++count;
      }

      ROS_INFO_STREAM("[Talker] Sending -> " << msg.data.c_str());

      /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
      */

      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}


