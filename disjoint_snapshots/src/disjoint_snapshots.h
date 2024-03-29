#ifndef DISJOINT_SNAPSHOTS_H
#define DISJOINT_SNAPSHOTS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include "ap_msgs/PauseResumePatroller.h"

class disjoint_snapshots {
private:
	long int counter;
	long int pcd_counter;
	std::string folder;
	ros::ServiceClient& client;
	double snapped_x, snapped_y;
	double snapped_angle;
	double x, y;
	double angle;
	double amcl_x, amcl_y;
	double amcl_angle;
	Eigen::Quaterniond amcl_quat;
	double fov;
	double dist;
	bool snapshot;
	bool pcd_snapshot;
	bool first;
	ros::Timer timer;
	ros::NodeHandle& n;
public:
	void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void geometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void allow_snapshot(const ros::TimerEvent& e);
	void image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                    	const sensor_msgs::Image::ConstPtr& rgb_msg);
	void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	disjoint_snapshots(ros::NodeHandle& n, ros::ServiceClient& client, const std::string& folder, double dist);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#endif // DISJOINT_SNAPSHOTS_H
