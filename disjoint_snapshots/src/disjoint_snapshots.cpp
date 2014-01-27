#include "disjoint_snapshots.h"

disjoint_snapshots::disjoint_snapshots(ros::NodeHandle& n, ros::ServiceClient& client, const std::string& folder, double dist) :
	n(n), client(client), folder(folder), dist(dist), counter(0), pcd_counter(0), snapshot(false), pcd_snapshot(false), first(true)
{
	/*typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/head_xtion/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, "/head_xtion/rgb/image_color", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(&disjoint_snapshots::image_callback, this);
	ros::Subscriber sub = n.subscribe("/odom", 1, &disjoint_snapshots::geometry_callback, this);*/

	//client = n.serviceClient<ap_msgs::PauseResumePatroller>("pause_resume_patroller");

	// compute FOV for x
	double cx = 326.0; // center in image plane for x
	double fx = 566.0; // focal length for x
	fov = 2.0*atan(cx/fx);
}

void disjoint_snapshots::geometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    Eigen::Quaterniond q(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
	Eigen::Vector3d ea = q.matrix().eulerAngles(0, 1, 2);
	double angle = ea(0);

	double xd = x - snapped_x;
	double yd = y - snapped_y;
	double ad = fmod(fabs(angle - snapped_angle), M_PI); // think this through

	if (first) {
		first = false;
	}
	else if (sqrt(xd*xd + yd*yd) < dist && ad < fov) {
	    return;
	}

	std::cout << "Distance: " << sqrt(xd*xd + yd*yd) << std::endl;
	std::cout << "Angle distance: " << ad << std::endl;
	std::cout << "Taking snapshot now..." << std::endl;

	snapped_x = x;
	snapped_y = y;
	snapped_angle = angle;
	
	ap_msgs::PauseResumePatroller srv;
	bool success = client.call(srv);
	std::cout << "Result: " << srv.response.result << std::endl;
	if (!success) {
	    ROS_ERROR("Couldn't pause the patroller because pause_resume_patroller service not available.");
		return;
	}
	if (srv.response.result != "paused") {
		ROS_ERROR("The patroller was resumed instead of paused.");
		return;
	}

	timer = n.createTimer(ros::Duration(3.0), &disjoint_snapshots::allow_snapshot, this, true);
}

void disjoint_snapshots::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	amcl_x = msg->pose.pose.position.x;
    amcl_y = msg->pose.pose.position.y;
	geometry_msgs::Quaternion geom_quat = msg->pose.pose.orientation;
	amcl_quat = Eigen::Quaterniond(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
	Eigen::Vector3d ea = amcl_quat.matrix().eulerAngles(0, 1, 2);
	amcl_angle = ea(0);
}

void disjoint_snapshots::allow_snapshot(const ros::TimerEvent& e)
{
	std::cout << "Allowing a snapshot!" << std::endl;
	snapshot = true;
	pcd_snapshot = true;
}

// called by the synchronizer, always with depth + rgb
void disjoint_snapshots::image_callback(const sensor_msgs::Image::ConstPtr& depth_msg,
                    const sensor_msgs::Image::ConstPtr& rgb_msg)
{
	if (!snapshot) {
		return;
	}
	snapshot = false;

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

	ap_msgs::PauseResumePatroller srv;
	if (!client.call(srv)) {
		ROS_ERROR("Couldn't resume the patroller because pause_resume_patroller service not available.");
	}
	if (srv.response.result == "paused") {
		ROS_INFO("Controller was not resumed, trying again!");
		client.call(srv);
	}
    
    ++counter;
}

// here we also save the current position to be able to draw it on the map
void disjoint_snapshots::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	if (!pcd_snapshot) {
		return;
	}
	pcd_snapshot = false;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
	char buffer[250];
    sprintf(buffer, "%s/cloud%06ld.pcd", folder.c_str(), pcd_counter);
	pcl::io::savePCDFileBinary(buffer, cloud);
	
	std::ofstream posfile;
	sprintf(buffer, "%s/position%06ld.txt", folder.c_str(), pcd_counter);
    posfile.open(buffer);
    posfile << amcl_x << " " << amcl_y << " " << amcl_angle << " " <<
			amcl_quat.x() << " " << amcl_quat.y() << " " <<
			amcl_quat.z() << " " << amcl_quat.w();
    posfile.close();
	
	++pcd_counter;
}
