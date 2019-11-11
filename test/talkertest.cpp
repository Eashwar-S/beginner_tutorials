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
 * @file talkertest.cpp
 *
 * @author Eashwar Sathyamurthy
 *
 * @brief INtegration test case (Google Test framework) for CLient service.
 *
 * @version 1
 *
 * @date 2019-11-09
 *
 * This file defines a test case to test the Client node.
 */
#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "beginner_tutorials/DisplayService.h"

/**
 *@brief Integration Test for testing Client node
 *
 *This test checks the Client service by calling the service
 *and checks if the service is processed or not by testing the response.
 *
 */
TEST(talkertest, testingCLientService) {
  ros::NodeHandle nh;
  /// Creating a service object to access service file contents
  beginner_tutorials::DisplayService displayService;
  /// Setting request
  displayService.request.desiredMessage = "Client passed the test";
  ros::ServiceClient client = nh
      .serviceClient<beginner_tutorials::DisplayService>(
          "changing_talker_output");
  /// Calling the client
  bool pass = client.call(displayService);
  EXPECT_TRUE(pass);
  if (pass) {
    /// Checking if request is processed or not
    EXPECT_EQ(displayService.request.desiredMessage,
              displayService.response.outputMessage);
  }
}

/**
 *@brief main method to run all test cases
 *
 *This method to used to run the tests using google framework.
 *
 */

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talkertest");
  return RUN_ALL_TESTS();
}
