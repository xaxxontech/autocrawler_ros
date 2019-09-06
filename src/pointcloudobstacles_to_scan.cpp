#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>

#include <chrono>
#include <ctime> 
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher pub;

int frame = 0;

void callback(const PointCloud::ConstPtr& msg)
// void callback(PointCloud::Ptr msg)
{
	// printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
	// BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
		// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		
	// frame ++;
	// if (frame % 10 != 0) return;
		 
	auto start = std::chrono::system_clock::now();
    
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	
	Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
	seg.setAxis(axis);
	seg.setEpsAngle(  10.0f * (M_PI/180.0f) );
	seg.setMaxIterations(500); 

	seg.setInputCloud (msg);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		ROS_INFO("Could not estimate a planar model for the given dataset.");
		return;
	}
	
	// from https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(msg);
	extract.setIndices(inliers);
	extract.setNegative(true); // so output is everything BUT floor plane

	PointCloud::Ptr outputcloud(new PointCloud);
	extract.filter(*outputcloud);
	// extract.filter(msg);

	auto end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	std::cout << "time: " << elapsed_seconds.count() << "s\n"; 
	
	// sensor_msgs::PointCloud2 output;
	// pcl_conversions::fromPCL(*outputcloud, output);
	pub.publish(outputcloud);
	// pub.publish(msg);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloudobstaclestoscan");
	ROS_INFO("rosinfo INIT pointcloudobstaclestoscan");
	printf("%s\n","init print");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/color/points", 1, callback);
	pub = nh.advertise<PointCloud> ("points2", 1);
	ros::spin();
}
