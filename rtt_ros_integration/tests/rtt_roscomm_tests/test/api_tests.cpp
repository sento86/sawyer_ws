/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <rtt/os/startstop.h>
#include <ocl/DeploymentComponent.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/scripting/Scripting.hpp>

#include <gtest/gtest.h>

boost::shared_ptr<OCL::DeploymentComponent> deployer;
boost::shared_ptr<RTT::Scripting> scripting_service;

TEST(BasicTest, Import) 
{
  // Import rtt_ros plugin
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_ros", "" ));
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_roscomm", "" ));
}

TEST(BasicTest, ImportTypekit) 
{
  // Import rtt_std_msgs typekit
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_msgs", "" ));
  EXPECT_TRUE(scripting_service->eval("var ConnPolicy float_out = ros.topic(\"float_out\")"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  deployer = boost::make_shared<OCL::DeploymentComponent>();
  scripting_service = deployer->getProvider<RTT::Scripting>("scripting");

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);
  
  return RUN_ALL_TESTS();
}

