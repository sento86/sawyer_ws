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

#include <netft_rdt_driver/netft_hardware_interface.h>


#define DEFAULT_IP_ADDRESS        "192.168.1.1"


namespace netft_rdt_driver
{
  NetftHardwareInterface::NetftHardwareInterface()
  {
    // Get parameters from the server
    std::string topic, frame_id;
    nh_.param(std::string("address"), ip_address_, std::string(DEFAULT_IP_ADDRESS));
    if (!nh_.hasParam("address"))
      ROS_WARN_STREAM("Parameter [address] not found, using default: " << ip_address_);
    
    nh_.param(std::string("topic"), topic, std::string("raw"));
    if (!nh_.hasParam("topic"))
      ROS_WARN_STREAM("Parameter [topic] not found, using default: " << topic);
    
    nh_.param(std::string("frame_id"), frame_id, std::string("base_link"));
    if (!nh_.hasParam("frame_id"))
      ROS_WARN_STREAM("Parameter [frame_id] not found, using default: " << frame_id);

    // Set initial values
    force_[0] = 0;
    force_[1] = 0;
    force_[2] = 0;
    torque_[0] = 0;
    torque_[1] = 0;
    torque_[2] = 0;
    
    // Connect the handle with the hardware interface
    hardware_interface::ForceTorqueSensorHandle ft_sensor_handle(topic, frame_id, force_, torque_);
    ft_sensor_interface_.registerHandle(ft_sensor_handle);
    
    // Setup Real-Time Publishers
    diag_publisher_.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh_, "diagnostics", 2));
    // Will publish diagnostics every second
    diag_pub_duration_ = ros::Duration(1.0);
    diag_array_.status.reserve(1);
    
    registerInterface(&ft_sensor_interface_);
    ROS_INFO("Registered ForceTorqueSensorInterface");
  }
  
  bool NetftHardwareInterface::start()
  {
    try {
      netft_.reset(new netft_rdt_driver::NetFTRDTDriver(ip_address_));
    }
    catch (...) {
      ROS_ERROR("Failed to connect to netft with IP address: %s. Please DOUBLE CHECK the connection with the sensor", ip_address_.c_str());
      return false;
    }  
    return true;
  }
  
  void NetftHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
  {
    geometry_msgs::WrenchStamped data;
    if (netft_->waitForNewData())
    {
      netft_->getData(data);
      force_[0] =   data.wrench.force.x;
      force_[1] =   data.wrench.force.y;
      force_[2] =   data.wrench.force.z;
      torque_[0] =  data.wrench.torque.x;
      torque_[1] =  data.wrench.torque.y;
      torque_[2] =  data.wrench.torque.z;
    }
    
    ros::Time current_time(ros::Time::now());
    if ( (current_time - last_diag_pub_time_) > diag_pub_duration_ )
    {
      diag_array_.status.clear();
      netft_->diagnostics(diag_status_);
      diag_array_.status.push_back(diag_status_);
      diag_array_.header.stamp = ros::Time::now();
      if(diag_publisher_->trylock())
      {
        diag_publisher_->msg_.header.stamp = ros::Time::now();
        diag_publisher_->msg_.status = diag_array_.status;
        diag_publisher_->unlockAndPublish();
      }
      last_diag_pub_time_ = current_time;
    }
    
    
  }

} // namespace
