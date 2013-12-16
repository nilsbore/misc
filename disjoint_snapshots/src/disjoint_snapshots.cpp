#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

long int counter;
std::string folder;

double snapped_x, snapped_y;
double snapped_angle;
double x, y;
double angle;
double fov;
double dist;

void geometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    Eigen::Quaterniond q(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
	Eigen::Vector3d ea = q.matrix().eulerAngles(0, 1, 2);
	angle = ea(0);
}

// called by the synchronizer, always with depth + rgb
void image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                    const sensor_msgs::Image::ConstPtr& rgb_msg)
{
    double xd = x - snapped_x;
    double yd = y - snapped_y;
    double ad = fmod(fabs(angle - snapped_angle), M_PI); // think this through
	
    if (sqrt(xd*xd + yd*yd) < dist && ad < fov && counter > 0) {
        return;
    }

	std::cout << "Distance: " << sqrt(xd*xd + yd*yd) << std::endl;
	std::cout << "Angle distance: " << ad << std::endl;
	std::cout << "Taking snapshot now..." << std::endl;

	snapped_x = x;
	snapped_y = y;
	snapped_angle = angle;
    
    // convert message to opencv images for saving
    boost::shared_ptr<sensor_msgs::Image> depth_tracked_object;
	cv_bridge::CvImageConstPtr depth_cv_img_boost_ptr;
	try {
		depth_cv_img_boost_ptr = cv_bridge::toCvShare(*depth_msg, depth_tracked_object);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	boost::shared_ptr<sensor_msgs::Image> rgb_tracked_object;
	cv_bridge::CvImageConstPtr rgb_cv_img_boost_ptr;
	try {
		rgb_cv_img_boost_ptr = cv_bridge::toCvShare(*rgb_msg, rgb_tracked_object);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
    // save images to folder
    std::vector<int> compression;
    compression.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression.push_back(0); // no compression
    
    char buffer[250];
    
    sprintf(buffer, "%s/rgb%06ld.png", folder.c_str(), counter);
    cv::imwrite(buffer, rgb_cv_img_boost_ptr->image, compression);
    
    sprintf(buffer, "%s/depth%06ld.png", folder.c_str(), counter);
    cv::imwrite(buffer, depth_cv_img_boost_ptr->image, compression);
    
    ++counter;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disjoint_snapshots");
	ros::NodeHandle n;
	
	// topic of input cloud
    if (!n.hasParam("/disjoint_snapshots/folder")) {
        ROS_ERROR("Could not find parameter folder.");
        return -1;
    }
    n.getParam("/disjoint_snapshots/folder", folder);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/head_xtion/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/head_xtion/rgb/image_color", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(&image_callback);
	ros::Subscriber sub = n.subscribe("/odom", 1, &geometry_callback);
    counter = 0;

	// compute FOV for x
	double cx = 326.0; // center in image plane for x
	double fx = 566.0; // focal length for x
	fov = 2.0*atan(cx/fx);

	// at what distance should we capture a new snapshot?
	dist = 2.0; // m
    
    ros::spin();
	
	return 0;
}
