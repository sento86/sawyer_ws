/*
  Point cloud transformation
  extension of manipulate_cloud
  code aims to transform the point cloud from the camera frame to the base frame of the robot
  luke ramos, 2018
*/

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

// pcl: for voxel downsampling
#include <pcl/filters/voxel_grid.h>
// pcl: for plane manipulation
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// pcl: normal estimation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// pcl: normal estimation temporary
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h> // needs odered cloud
#include <pcl/features/normal_3d.h>							// normals at each 3D point
#include <pcl/features/feature.h>								// k search
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
//#include <pcl/visualization/image_viewer.h>

// transformation inclusions
//#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

class CloudProcessor
{
public:
  CloudProcessor(ros::NodeHandle _nh):
    nh(_nh)
  {
    sub        = nh.subscribe<sensor_msgs::PointCloud2> ("/sawyer/rgbd_sensor/sawyer/rgbd_sensor_camera/depth/points", 10, &CloudProcessor::cloudCallBack,this);
    tfCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/surface", 1);
    markerPub  = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    polishPub  = nh.advertise<geometry_msgs::PoseStamped>("/polishTouchPose", 1);

    double threshold = 0.05;
    /** we segment a planar surface  */
    segmentation.setOptimizeCoefficients (true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold (threshold);
    coefficients = pcl::ModelCoefficients ();
    extract.setNegative (false);
    newCloudReceived = false;

    ros::Rate rate(10);
    while(nh.ok())
    {
      if(newCloudReceived)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface = plannerSegmentation(inCloud);
        Eigen::Vector4f centroid;
        compute3DCentroid(*surface,centroid);
        publishMarker(centroid);

//        ROS_INFO("Before, the frame id was %s",surface->header.frame_id.c_str() );
        surface->header.frame_id = std::string("sawyer/rgbd_sensor/camera_depth_optical_frame");
        pcl_ros::transformPointCloud("/base", *surface, *cloudOut, listener);

        sensor_msgs::PointCloud2 cloudOutPub;
        pcl::toROSMsg(*cloudOut, cloudOutPub);
        //cloudOutPub.header.frame_id = "/base";
//        ROS_INFO("After, the frame id was %s",cloudOutPub.header.frame_id.c_str());
        tfCloudPub.publish (cloudOutPub);
        newCloudReceived = false;
      }
      rate.sleep();
      ros::spinOnce();
    }
  }
  ~CloudProcessor(){}

  void publishMarker(Eigen::Vector4f centroid)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/sawyer/rgbd_sensor/camera_depth_optical_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2] - 0.08; // -0.15 for vicent setup. -0.08 for buff_l setup
    std::cout<<"\n2- Coefficient Values x:"<<coefficients.values[0]<< " y:"<<coefficients.values[1]<<" z:"<<coefficients.values[2];
    tf::Vector3 axisVector(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    tf::Vector3 vectorUp(0.0, 0.0, 1.0);
    tf::Vector3 rightVector = axisVector.cross(vectorUp);
    rightVector.normalized();
    tf::Quaternion q(rightVector, -1.0*acos(axisVector.dot(vectorUp)));
    q.normalize();
    geometry_msgs::Quaternion planeOrientation;
    tf::quaternionTFToMsg(q, planeOrientation);
    marker.pose.orientation = planeOrientation;

    marker.scale.x = 0.25;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.1);
    markerPub.publish(marker);
    polishTouchPose.pose.position    = marker.pose.position;
    polishTouchPose.pose.orientation = marker.pose.orientation;
    polishTouchPose.header.frame_id = "sawyer/rgbd_sensor/camera_depth_optical_frame";
    polishPub.publish(polishTouchPose);
  }

  void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
  {
    boost::mutex::scoped_lock lock(mutex);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInCopy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloudIn,*cloudInCopy);
    inCloud = cloudInCopy;
    newCloudReceived = true;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr plannerSegmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    boost::mutex::scoped_lock lock(mutex);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;
    //first apply a filter
    grid.setFilterFieldName ("z");
    grid.setFilterLimits (0.0f, 1.0f);
    grid.setInputCloud(cloud);
    grid.setLeafSize (0.01f, 0.01f, 0.01f);
    grid.filter(*filteredCloud);
    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height << " data points." << " w:"<<filteredCloud->width <<" h:"<<filteredCloud->height << std::endl;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());

    segmentation.setInputCloud(filteredCloud);
    segmentation.segment(*inliers, coefficients);
    extract.setInputCloud(filteredCloud);
    extract.setIndices(inliers);
    extract.filter(*tempCloud);
    std::cout<<"\n1- Coefficient Values x:"<<coefficients.values[0]<< " y:"<<coefficients.values[1]<<" z:"<<coefficients.values[2];

    return tempCloud;
  }

private:
  ros::Publisher tfCloudPub,trasferedCloudPub,polishPub,markerPub;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ModelCoefficients coefficients;
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  boost::mutex mutex;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr inCloud;
  geometry_msgs::PoseStamped polishTouchPose;
  bool newCloudReceived;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_transform");
  ros::NodeHandle nh;

  CloudProcessor cp(nh);

  ros::spin();
  return 0;
}

