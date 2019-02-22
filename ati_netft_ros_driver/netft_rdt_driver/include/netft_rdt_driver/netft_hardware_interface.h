/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, CRI Lab at Nanyang Technological University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Francisco Suarez Ruiz
   Desc:   ros_control hardware interface layer for netft usign the rdt driver
*/

#ifndef DENSO_CONTROL__DENSO_HARDWARE_INTERFACE_
#define DENSO_CONTROL__DENSO_HARDWARE_INTERFACE_

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/force_torque_sensor_interface.h>

// netft_rdt_driver
#include "netft_rdt_driver/netft_rdt_driver.h"


namespace netft_rdt_driver
{

class NetftHardwareInterface : public hardware_interface::RobotHW
{
  public:
    NetftHardwareInterface();
    bool start();
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period)  {};
    void stop()     {};
    void cleanup()  {};

  private:
    // State
    ros::NodeHandle nh_;
    
    // netft_rdt_driver
    std::string   ip_address_;
    boost::scoped_ptr<netft_rdt_driver::NetFTRDTDriver> netft_;    
    
    // Diagnostics
    boost::scoped_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> >  diag_publisher_;
    diagnostic_msgs::DiagnosticArray            diag_array_;
    diagnostic_updater::DiagnosticStatusWrapper diag_status_;
    ros::Time last_diag_pub_time_;
    ros::Duration diag_pub_duration_;
    
    // Interfaces
    hardware_interface::ForceTorqueSensorInterface    ft_sensor_interface_;

    // Data is read from/written to these internal variables
    double force_[3];
    double torque_[3];
};

} // namespace

#endif
