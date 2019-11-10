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
 * @brief test cases (Google Test framework) for talker node.
 *
 * @param display Parameter giving status as to which message to publish.
 *
 * @param debug Default publisher frequency.
 *
 * @version 1
 *
 * @date 2019-11-09
 *
 *
 */
#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cstdlib>
#include "beginner_tutorials/DisplayService.h"

TEST(talkertest, testingCLientService) {
  ros::NodeHandle nh;
  beginner_tutorials::DisplayService displayService;
    displayService.request.desiredMessage = "Client passed the test";
  ros::ServiceClient client = nh
        .serviceClient<beginner_tutorials::DisplayService>(
            "changing_talker_output");
  bool pass = client.call(displayService);
  EXPECT_TRUE(pass);
  if(pass)
    EXPECT_EQ(displayService.request.desiredMessage, displayService.response.outputMessage);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "talkertest");
  return RUN_ALL_TESTS();
}
