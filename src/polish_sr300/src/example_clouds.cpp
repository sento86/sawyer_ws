#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// pcl: for voxel downsampling
#include <pcl/filters/voxel_grid.h> 
// pcl: for plane manipulation
#include <pcl/sample_consensus/model_types.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>


ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform actual downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "polish_sr300");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("fullsize_cloud", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, cloud_cb);

  // Create a ROS publisher of the downsampled output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_cloud", 1);

  // Spin
  ros::spin ();
}


