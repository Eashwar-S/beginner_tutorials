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
 * @brief A C++ publisher node for sending messages over the ROS system.
 *
 * @version 1
 *
 * @date 2019-10-26
 *
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "beginner_tutorials/DisplayService.h"

bool display = false;
std::string output;
int debug = 0;
bool newMessage(beginner_tutorials::DisplayService::Request &request,
                beginner_tutorials::DisplayService::Response &response) {
  ROS_WARN_STREAM("Modifying the Custom Message of the Publisher");
  if (request.desiredMessage.size() > 0) {
    response.outputMessage = request.desiredMessage;
    output = response.outputMessage;
    ROS_INFO_STREAM("Modified Talker's message to :" << response.outputMessage);
    if (debug == 0) {
      ROS_DEBUG_STREAM("Is debug variable == " << 0);
      display = !display;
      debug++;
    }
    return true;
  } else {
    ROS_ERROR_STREAM(
        "Publisher message modification Unsuccessful. No new message received from the client");
    return false;
  }
  return false;
}
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

  ros::init(argc, argv, "Server");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle nh_;
  ros::Publisher chatter_pub = nh_.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer displayService_ = nh_.advertiseService(
      "changing_talker_output", &newMessage);

  ros::Rate loop_rate(10);
  auto count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::stringstream ss;
    if (display) {
      ROS_INFO_STREAM_ONCE("Publisher is publishing the new Message");

      ss << output << ": Line : " << count;
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
      ss << "Hello World: Line : " << count;
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
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
