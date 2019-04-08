// Code to manipulate pointclouds
// Key aims: downsample, smoothing, generate surface normals
// jaime valls miro, 2018

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
<<<<<<< HEAD
=======

>>>>>>> 6c8961dcdd621074eb7576fb806b8f2c5c29d3ac

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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// 3 published topics
ros::Publisher pub;			// simple downsample
ros::Publisher pub_mls;		// mls resampling - costly!
ros::Publisher tf_pub;		// cloud transformation

// ******************************************************************
// Estimate a set of surface normals for all the points in the 
// (unorganized) input cloud. 
// No MLS. PCL only
//
void
PCL_estimate_cloud_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_in)
{
//	std::cout << "PCL surface normals" << std::endl;
	int pcl_viewer = 0; // display data on native pcl viewer - so I can see normals! 1 = show viewer. 0 = dont show viewer
	// showing the viewer will pause the script and the rostopic will not publish continuously
	// disabling the viewer will allow the rostopic to keep publishing and will make subscribing

	// calculate normals per point, use surrounding points 
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (pcl_cloud_in);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr pcl_cloud_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  // number of neighbouring points used to calculate normals for each point (surface)
  ne.setKSearch (100);
	//ne.setRadiusSearch (0.01);
  ne.compute (*pcl_cloud_normals_ptr);
  
  // visualize cloud and normals in native pcl viewer
  if (pcl_viewer)
  {
		pcl::visualization::PCLVisualizer viewer("cloud (pcl) viewer");
		viewer.setBackgroundColor (0.0, 0.0, 0.0);
  	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> pcl_rgb_handler(pcl_cloud_in);
  	viewer.addPointCloud<pcl::PointXYZRGB> (pcl_cloud_in, pcl_rgb_handler, "sr300 cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sr300 cloud");
  	viewer.addCoordinateSystem (0.05);
//  viewer.initCameraParameters ();
		// display every 100th normal at length of 0.1
		viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pcl_cloud_in, pcl_cloud_normals_ptr, 100, 0.1);
//		viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::PointNormal>(pcl_cloud_in, pcl_cloud_normals_ptr, 100, 0.1);  
//  while (!viewer.wasStopped ())
  	  viewer.spin ();
  }
   
	// convert to ROS msg and publish in topic

	// Copy point normals to XYZ cloud to push out - unfortunately don't seem to be
	// able to visualize normals (rviz), but I send on topic msgs anyway ... some day, somehow I'll use them, I hope.
	
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
	
	// This was the original method for publishing the point cloud, but was not publishing normal data.
	// pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>); 
	// copyPointCloud(*pcl_cloud_in, *normals_cloud); // ideally ne, with normals too! TODO
	
	// combining raw pointcloud containing PointXYZRGB (point coord and colour) 
	// with pointcloud containing Normal (only normal data per point, no coord data)
	// pcl::concatenateFields (*raw point cloud without normal, *point cloud containing normals, *point cloud with xyz, rgb and normal);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*pcl_cloud_in, *pcl_cloud_normals_ptr, *cloud_with_normals);

	// NOTE: I should be able to advertise PCL native objects pcl::PointCloud<pcl::PointXYZ>
	// but it does not work ... :( 

  sensor_msgs::PointCloud2 ros_cloud_out;
	pcl::toROSMsg(*cloud_with_normals, ros_cloud_out);

	// Publish the data
  pub.publish (ros_cloud_out); 


    // ********** POINT CLOUD TRANSFORM **********
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
  tf::Transform tform;
//	std::string* error_msg;
//	pcl_ros::transformPointCloud pcl_transform;

    // Perform point cloud transformation

  ros::Time now = ros::Time::now();
//    ros::Time now = ros::Time(0);
//  ros::Time now = ros::Time(0.0);
//  ros::Time now = ros::Time(0.1);
//	ros::Time now = ros::Time(2150.477000000);

//	tf::Transformer::canTransform("/right_arm_base_link", "/depth_link", now, error_msg);

//	listener.waitForTransform("/right_arm_base_link", "/depth_link", now, ros::Duration(5.0), ros::Duration(2));//, error_msg); // base on line 193 of /tf/tf.h
//	listener.lookupTransform("/right_arm_base_link", "/depth_link", now, transform); // based on line 129 /tf/tf.h


  try{
//		ros::Time now = ros::Time(0);
    listener.waitForTransform("/right_harnd", "/depth_link", now, ros::Duration(1.0), ros::Duration(0.001)); // base on line 193 of /tf/tf.h
    listener.lookupTransform("/right_hand", "/depth_link", now, transform); // based on line 129 /tf/tf.h
//		listener.waitForTransform("/right_hand", "/depth_link", now, ros::Duration(10.0), ros::Duration(0.01));
//		listener.lookupTransform("/right_hand", "/depth_link", now, transform);
//    std::cout << transform << std::endl;
    std::cout << "Performing transformation" << std::endl;
    }
  catch(tf::LookupException){ //tf2::ExtrapolationException
    std::cerr << "Transform lookup failed" << std::endl;
    }/*
  catch(tf2::ExtrapolationException){
    std::cerr << "Extrapolation required" << std::endl;
    }*/

//	listener.lookupTransform("/right_arm_base_link", "/depth_link", ros::Time(0), transform); // based on line 129 /tf/tf.h
//	listener.lookupTransform("/depth_link", "/right_arm_base_link", ros::Time(0), transform); // based on line 129 /tf/tf.h, but slight variation for experimentation
//	listener.lookupTransform("/right_arm_base_link", ros::Time(0), "/depth_link", ros::Time(0), , transform);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//  	tf::TransformListener::lookupTransform ("/base", "/depth_link", ros::Time::now(), tf::TransformListener (ros::Duration(5.0), true));

//	pcl_ros::transformPointCloud("/right_arm_base_link", ros::Time::now(), *cloud_with_normals, "/depth_link", *trans_cloud, listener); // based on line 78 /pcl_ros/transforms.h
  pcl_ros::transformPointCloud("/base", *cloud_with_normals, *trans_cloud, listener); // based on line 131 /pcl_ros/transforms.h
//	pcl_ros::transformPointCloud(*cloud_with_normals, *trans_cloud, listener); // based on line 131 /pcl_ros/transforms.h
//	pcl_ros::transformPointCloud("/right_arm_base_link", transform1, *cloud_with_normals, *trans_cloud); // based on line 143 /pcl_ros/transforms.h

  /**********************
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("right_arm_base_link", "depth_link", ros::Time(0));
    std::cout << transformStamped << std::endl;
    tf2::doTransform(*cloud_with_normals, *trans_cloud, transformStamped);
    }
  catch(tf2::TransformException &ex){
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
//		continue;
    }
  **********************/

    // Publish transformed point cloud
  std::cout << "Publishing transformed point cloud" << std::endl;
  sensor_msgs::PointCloud2 ros_trans_out;
  pcl::toROSMsg( *trans_cloud, ros_trans_out);
  tf_pub.publish (ros_trans_out);

}

// ******************************************************************
// Resample MLS filter to smooth clouds via surface normal estimation 
// with a Moving Least Squares (MLS) algorithm
void
MLS_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_in)
{
//  std::cout << "MLS" << std::endl;

	// Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

	// Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

	// Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
  mls.setComputeNormals (true);

	// Set parameters
  mls.setInputCloud (pcl_cloud_in);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.3); // 0.03

	// Reconstruct
	mls.process (mls_points);
	
	// Copy PointNormal mls_points to XYZ cloud to push out - unfortunately don't seem to be
	// able to visualize normals (rviz), but I send on topic msgs anyway ... some day, somehow I'll use them, I hope.
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>); 
	copyPointCloud(mls_points, *mls_cloud);
	
	// Convert mls cloud to ROS msg  
	sensor_msgs::PointCloud2 ros_cloud_out;
	pcl::toROSMsg(*mls_cloud, ros_cloud_out);
	
	// Publish the data
	pub_mls.publish (ros_cloud_out);
	
//  std::cerr << "Back from MLS" << std::endl;
}


// *************************************************
// Call-back on new cloud: clean and downsample data
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
	// Original cloud size 
// std::cerr << "Cloud data coming in" << std::endl;

	// Convert the ROS sensor_msgs/PointCloud2 data to PCLPointCloud2
	pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2; 
	pcl_conversions::toPCL(*ros_cloud, *pcl_cloud);

	// Original cloud size 
  std::cerr << "raw PointCloud before filtering: " << pcl_cloud->width * pcl_cloud->height << " data points." << std::endl;

  // Perform actual downsampling filtering
  pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(pcl_cloud);
	pcl::PCLPointCloud2 pcl_cloud_downsampled;

  pcl::VoxelGrid<pcl::PCLPointCloud2> voxelgrid;
  voxelgrid.setInputCloud (pcl_cloud_ptr);
  // choose downsample, larger leaf size numbers, less points.
  // fully dependant on the actual cloud we are dealing with.  
  // it should really be a config parameter.
	voxelgrid.setLeafSize (0.002, 0.002, 0.002);
//  voxelgrid.setLeafSize (0.2, 0.2, 0.2);

  voxelgrid.filter (pcl_cloud_downsampled);

	// For some reason I need to go from pcl::PCLPointCloud2 to pcl::PointCloud<PointXYZ> to 
	// use the code to filter NaN...??
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_pre (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_clean (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::fromPCLPointCloud2 (pcl_cloud_downsampled, *pcl_cloud_pre);	
	
	// Filter NaNs from the cloud, otherwise mls fails
  for (size_t i = 0; i < pcl_cloud_pre->size (); ++i)
    if (pcl_isfinite (pcl_cloud_pre->points[i].x))		
      pcl_cloud_clean->push_back (pcl_cloud_pre->points[i]);
//			else
//				std::cout << "NaN";
  pcl_cloud_clean->header = pcl_cloud->header;
  pcl_cloud_clean->height = 1;
  pcl_cloud_clean->width = static_cast<uint32_t> (pcl_cloud_clean->size ());
	pcl_cloud_clean->is_dense = false;


	// Clean cloud size 
//	std::cerr << pcl_cloud_clean->width << endl;
//	std::cerr << pcl_cloud_clean->height << endl;
  std::cerr << "clean PointCloud : " << pcl_cloud_clean->width * pcl_cloud_clean->height << " data points." << std::endl;


	// Publish the clean data
	sensor_msgs::PointCloud2 ros_cloud_out;
	pcl::toROSMsg(*pcl_cloud_clean, ros_cloud_out);

	pub_mls.publish (ros_cloud_out);

	// Any further processing here:
	// Smooth surface via MLS
  // MLS_filter(pcl_cloud_clean);
  // Compute and visualize normals - note I pass on the 'clean' cloud (NaN-free), not the downsampled one
	PCL_estimate_cloud_normals(pcl_cloud_clean);
}

// *************************
// main spin
int
main (int argc, char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "polish_sr300");
  ros::NodeHandle nh;

// Alive? 
  std::cerr << "polish_pclT started " << std::endl;
  	
// Create a ROS subscriber for the raw (colour-registered) input point cloud from SR300
// Either refer to input cloud topic generically, for code readability, and then configure on running node,
// e.g. rosrun polish_sr300 polish_pclT fullsize_cloud:=/camera/depth_registered/points
// ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("fullsize_cloud", 1, cloud_cb);
// or simply hardcode topic here:
//	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
//	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, cloud_cb);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/depth1/depth/points", 1, cloud_cb);

//	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/point_cloud", 1, cloud_cb);

// Create a ROS publisher of the downsampled output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_cloud", 1);

// Create a ROS publisher of the MLS filetered output point cloud
	pub_mls = nh.advertise<sensor_msgs::PointCloud2> ("mls_cloud", 1);

// Create ROS publishder of the transformed point cloud
//	pub_trans = nh.advertise<sensor_msgs::PointCloud2> ("transformed_cloud", 1);

// test for indexing and extracting fields of point cloud
//	std::cerr << pub << std::endl;

// point cloud transformation
//	ros::Subscriber sub_down = nh.subscribe<sensor_msgs::PointCloud2> ("/downsampled_cloud", 1, transform_cb);
  tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("trans_cloud", 1);

// Spin
  ros::spin ();
}

