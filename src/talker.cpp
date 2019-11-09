/***************************************************************************
 * Copyright (c) 2019, Eashwar Sathyamurthy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/**
 * @file talker.cpp
 *
 * @author Eashwar Sathyamurthy
 *
 * @brief A C++ publisher node having server service to change the messages
 *        published by the publisher.
 *
 * @param display Parameter giving status as to which message to publish.
 *
 * @param debug Default publisher frequency.
 *
 * @version 1
 *
 * @date 2019-11-01
 *
 *
 */
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdlib>
#include "beginner_tutorials/DisplayService.h"

bool display = false;
int debug = 0;
int publisherFrequency = 10;

/**
 * @brief method to change talker node's publishing message.
 *
 * @param &request Reference to service file
 *
 * @param &response Reference to service file
 *
 * @return bool status of message change.
 *
 */
bool newMessage(beginner_tutorials::DisplayService::Request &request,
                beginner_tutorials::DisplayService::Response &response) {
  ROS_WARN_STREAM("Modifying the Custom Message of the Publisher");
  if (request.desiredMessage.size() > 0) {
    response.outputMessage = request.desiredMessage;
    ROS_INFO_STREAM("Modified Talker's message to :" << response.outputMessage);
    if (debug == 0) {
      ROS_DEBUG_STREAM("Is debug variable == " << 0);
      display = !display;
      debug++;
    }
    return true;
  } else {
    ROS_ERROR_STREAM("Publisher message modification Unsuccessful. "
        "No new message received from the client");
    return false;
  }
  return false;
}

/**
 * @brief Main function
 *
 * @param argc Gives number of command line arguments passed
 *  including output file name.
 *
 * @param argv is array of character pointers
 *
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

  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle nh_;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  /// Creating a client service object
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
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
  ros::Publisher chatter_pub = nh_.advertise<std_msgs::String>("chatter", 1000);
  /// Creating server service object
  ros::ServiceServer displayService_ = nh_.advertiseService(
      "changing_talker_output", newMessage);
  if (argc == 2) {
    ROS_INFO_STREAM(
        "The publisher will publish messages at" " frequency " << std::atoi(argv[1]) << " Hertz");
    /// converting charater pointer to interger
    publisherFrequency = std::atoi(argv[1]);
  } else if (argc == 1) {
    ROS_WARN_STREAM(
        "No publisher frequency specified. Using default frequency");
  } else {
    ROS_ERROR_STREAM("Invalid Number of arguments");
  }
  ros::Rate loop_rate(publisherFrequency);
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    if (display) {
      ROS_INFO_STREAM_ONCE("Publisher is publishing the new Message");

      ss << "This is Eashwar : Line : " << count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      ++count;
      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      chatter_pub.publish(msg);
    } else {
      ss << "ENPM808X Assignment: Line : " << count;
      msg.data = ss.str();
      ++count;
      ROS_INFO_STREAM(msg.data.c_str());

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      chatter_pub.publish(msg);
    }
    transform.setOrigin(tf::Vector3(1.5, 3.0, 0.0));
    tf::Quaternion q;
    q.setRPY(1, 2, M_PI / 2);
    transform.setRotation(q);
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
