#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>

int counter;
std::string folder;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::string file = folder + "/cloud" + boost::lexical_cast<std::string>(counter)  + ".pcd";
    std::cout << file << std::endl;
	pcl::io::savePCDFileBinary(file, cloud);
	++counter;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stream_to_pcd");
	ros::NodeHandle n;
	
	// topic of input cloud
    if (!n.hasParam("/stream_to_pcd/folder")) {
        ROS_ERROR("Could not find parameter folder.");
        return -1;
    }
    n.getParam("/stream_to_pcd/folder", folder);
    
	ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1, callback);
    counter = 0;
    
    ros::spin();
	
	return 0;
}
