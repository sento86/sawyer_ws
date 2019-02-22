#ati_netft_ros_driver

This is the driver for the ATI NET/FT box that is used to connect to ATI F/T sensor. This repo is ported from https://code.ros.org/svn/wg-ros-pkg/stacks/netft/trunk. 
It has been catkinized and updated for ROS Indigo. This ROS driver only supports communciation with the NET/FT device using the Raw Data Transfer (RDT) protocol (using UDP packets) over Ethernet. The NET/FT box can also be interfaced using Ethernet/IP or CAN but these modes are not supported by the ROS driver. 

## Usage

Use the ROS node in the package as a stand-alone node for publishing F/T information. E.g.:

```
rosrun netft_rdt_driver netft_node --address 192.168.1.1
``` 

The NET/FT box defaults to 192.168.1.1 as its ip address but this can be changed using the browser-based interface.

See a more detailed tutorial here: http://wiki.ros.org/netft_rdt_driver/Tutorials/RunningNetFTNode

### Real-Time Hardware Interface

For applications that require to read from the netft in real-time, there is an implementation that uses a `hardware_interface::ForceTorqueSensorInterface` and a `force_torque_sensor_controller::ForceTorqueSensorController` from `ros_control` along side with OROCOS to achieve such behaviour.

This implementation is based in this [rtt_ros_control_example](https://github.com/skohlbr/rtt_ros_control_example).

There is a convenient launch file:
```
roslaunch netft_rdt_driver netft_rrt_controller.launch ip_address:=192.168.1.1
``` 

For more insights regarding this implementation please read this [useful discussion](https://github.com/ros-controls/ros_control/issues/130).
