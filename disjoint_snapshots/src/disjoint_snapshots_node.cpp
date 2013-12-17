#include "disjoint_snapshots.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "disjoint_snapshots_node");
	ros::NodeHandle n;
	
	// topic of input cloud
    if (!n.hasParam("/disjoint_snapshots_node/folder")) {
        ROS_ERROR("Could not find parameter folder.");
        return -1;
    }
	std::string folder;
    n.getParam("/disjoint_snapshots_node/folder", folder);
    
	ros::ServiceClient client = n.serviceClient<ap_msgs::PauseResumePatroller>("pause_resume_patroller");
    disjoint_snapshots d(n, client, folder, 2.0);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/head_xtion/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/head_xtion/rgb/image_color", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(&disjoint_snapshots::image_callback, &d);
	ros::Subscriber pcd_sub = n.subscribe("/head_xtion/registered/points", 1, &disjoint_snapshots::pointcloud_callback, &d);
	ros::Subscriber sub = n.subscribe("/odom", 1, &disjoint_snapshots::geometry_callback, &d);

	std::cout << "Spinning..." << std::endl;
    
    ros::spin();
	
	return 0;
}
