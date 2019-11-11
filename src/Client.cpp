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
 * @file Client.cpp
 *
 * @author Eashwar Sathyamurthy
 *
 * @brief A C++ node used to provide the service of the requesting the server
 *        service to modify the talker node's publishing message.
 *
 * @version 1
 *
 * @date 2019-11-01
 *
 *
 */
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "beginner_tutorials/DisplayService.h"
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

  ros::init(argc, argv, "Client");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;
  /// Setting Verbosity Levels
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  /// Creating a client service object
  ros::ServiceClient client = n
      .serviceClient<beginner_tutorials::DisplayService>(
          "changing_talker_output");
  /// Creating service file object
  beginner_tutorials::DisplayService displayService;
  ROS_INFO_STREAM("Requesting server service to change publisher message");
  displayService.request.desiredMessage = "This is Eashwar";
  /// Setting loop rate very low for better visualization of output.
  ros::Rate loop_rate(0.5);
  loop_rate.sleep();
  if (client.call(displayService)) {
    ROS_INFO_STREAM(
        "Successfully changed Publisher's message" " to "<< displayService.request.desiredMessage << " from ENPM808X Assignment");
    ros::spin();
  } else {
    ROS_FATAL_STREAM("Client call() not possible due to Server unavailability."
        " Please run the Server service.");
  }
  return 0;
}
