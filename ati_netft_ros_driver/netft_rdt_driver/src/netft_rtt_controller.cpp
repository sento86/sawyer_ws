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
   Desc:   rtt controller for netft usign the rdt driver
*/

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <boost/thread.hpp>

#include <netft_rdt_driver/netft_hardware_interface.h>
#include <controller_manager/controller_manager.h>


using namespace RTT;

class NetftRttController : public RTT::TaskContext{
  private:

    // Necessary components to run thread for serving ROS callbacks
    boost::thread non_rt_ros_queue_thread_;
    boost::shared_ptr<ros::NodeHandle> non_rt_ros_nh_;
    ros::CallbackQueue non_rt_ros_queue_;

    // The Netft hardware interface
    boost::shared_ptr<netft_rdt_driver::NetftHardwareInterface> hw_interface_;

    // The controller manager
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // For saving last update time, so period can be handed to controller manager
    ros::Time last_update_time_;

  public:
    NetftRttController(const std::string& name):
      TaskContext(name)
    {}

    ~NetftRttController()
    {}

  private:

    bool configureHook(){

      non_rt_ros_nh_.reset(new ros::NodeHandle(""));
      non_rt_ros_nh_->setCallbackQueue(&non_rt_ros_queue_);
      this->non_rt_ros_queue_thread_ = boost::thread( boost::bind( &NetftRttController::serviceNonRtRosQueue,this ) );

      hw_interface_.reset(new netft_rdt_driver::NetftHardwareInterface);
      hw_interface_->start();

      controller_manager_.reset(new controller_manager::ControllerManager(hw_interface_.get(), *non_rt_ros_nh_));

      last_update_time_ = rtt_rosclock::rtt_now();
      ROS_INFO("Finished configureHook");
      return true;
    }

    void updateHook(){

      // Get current system time (for timestamps of ROS messages)
      ros::Time now (rtt_rosclock::host_now());

      // Get guaranteed monotonic time for period computation
      ros::Time now_monotonic(rtt_rosclock::rtt_now());

      ros::Duration period (now_monotonic - last_update_time_);
      last_update_time_ = now_monotonic;

      hw_interface_->read(now, period);
      controller_manager_->update(now, period);
      hw_interface_->write(now, period);
    }
    
    void stopHook()
    {
      hw_interface_->stop();
    }

    void cleanupHook(){
      hw_interface_->cleanup();
      non_rt_ros_nh_->shutdown();
      non_rt_ros_queue_thread_.join();
    }

    void serviceNonRtRosQueue()
    {
      static const double timeout = 0.001;

      while (this->non_rt_ros_nh_->ok()){
        this->non_rt_ros_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

};
ORO_CREATE_COMPONENT(NetftRttController)
