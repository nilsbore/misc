#include <iostream>

#include "ros/ros.h"

#include "primitive_extraction/primitive.h"
#include "primitive_extraction/primitive_array.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>

ros::Publisher pub;
int previous_n;
bool display_plane_box;
double plane_r;
double plane_g;
double plane_b;

void write_plane_marker(visualization_msgs::Marker& marker, const primitive_extraction::primitive& primitive)
{
    if (display_plane_box) {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose.position.x = primitive.pose.position.x;
        marker.pose.position.y = primitive.pose.position.y;
        marker.pose.position.z = primitive.pose.position.z;
        marker.pose.orientation.x = primitive.pose.orientation.x;
        marker.pose.orientation.y = primitive.pose.orientation.y;
        marker.pose.orientation.z = primitive.pose.orientation.z;
        marker.pose.orientation.w = primitive.pose.orientation.w;
        marker.scale.x = 0.01;
        marker.scale.y = primitive.params[0];
        marker.scale.z = primitive.params[1];
    }
    else {
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        Eigen::Quaterniond quat;
        quat.setIdentity();
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
        marker.scale.x = 0.02;
        marker.points.resize(2*primitive.points.size());
        for (int i = 0; i < primitive.points.size()-1 ; ++i) {
            marker.points[2*i+2] = primitive.points[i];
            marker.points[2*i+3] = primitive.points[i+1];
        }
        marker.points[0] = primitive.points[primitive.points.size() - 1];
        marker.points[1] = primitive.points[0];
    }
    
    marker.color.a = 1.0;
    marker.color.r = plane_r;
    marker.color.g = plane_g;
    marker.color.b = plane_b;
    
}

void write_cylinder_marker(visualization_msgs::Marker& marker, const primitive_extraction::primitive& primitive)
{
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = primitive.pose.position.x;
    marker.pose.position.y = primitive.pose.position.y;
    marker.pose.position.z = primitive.pose.position.z;
    Eigen::Quaterniond q;
    q.x() = primitive.pose.orientation.x;
    q.y() = primitive.pose.orientation.y;
    q.z() = primitive.pose.orientation.z;
    q.w() = primitive.pose.orientation.w;
    Eigen::Quaterniond t(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()));
    q *= t;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 2.0*primitive.params[0];
    marker.scale.y = 2.0*primitive.params[0];
    marker.scale.z = primitive.params[1];
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}

void write_sphere_marker(visualization_msgs::Marker& marker, const primitive_extraction::primitive& primitive)
{
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = primitive.pose.position.x;
    marker.pose.position.y = primitive.pose.position.y;
    marker.pose.position.z = primitive.pose.position.z;
    marker.pose.orientation.x = primitive.pose.orientation.x;
    marker.pose.orientation.y = primitive.pose.orientation.y;
    marker.pose.orientation.z = primitive.pose.orientation.z;
    marker.pose.orientation.w = primitive.pose.orientation.w;
    marker.scale.x = 2.0*primitive.params[0];
    marker.scale.y = 2.0*primitive.params[0];
    marker.scale.z = 2.0*primitive.params[0];
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
}

void callback(const primitive_extraction::primitive_array::ConstPtr& msg)
{
    
    std::cout << "Tjena!" << std::endl;
    int n = msg->primitives.size();
    std::string camera_frame = msg->camera_frame;
    visualization_msgs::MarkerArray markers;
    
    if (n >= previous_n) {
        markers.markers.resize(n);
    }
    else {
        markers.markers.resize(previous_n);
    }
    
    for (int i = 0; i < n; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = camera_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace"; // what's this for?
        marker.id = i + 1;
        marker.action = visualization_msgs::Marker::ADD;
        std::string primitive_type = msg->primitives[i].type;
        if (primitive_type == "plane") {
            write_plane_marker(marker, msg->primitives[i]);
        }
        else if (primitive_type == "cylinder") {
            write_cylinder_marker(marker, msg->primitives[i]);
        }
        else if (primitive_type == "sphere") {
            write_sphere_marker(marker, msg->primitives[i]);
        }
        markers.markers[i] = marker;
    }
    
    for (int i = n; i < previous_n; ++i) {
        markers.markers[i].action = visualization_msgs::Marker::DELETE;
        markers.markers[i].header.frame_id = camera_frame;
        markers.markers[i].header.stamp = ros::Time();
        markers.markers[i].ns = "my_namespace"; // what's this for?
        markers.markers[i].id = i + 1;
    }
    
    previous_n = n;
    pub.publish(markers);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "primitives_to_markers");
    ros::NodeHandle n;
    
    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
	ros::NodeHandle pn("~");
	std::string input;
	pn.param<std::string>("input", input, std::string("/primitives"));
	std::string output;
	pn.param<std::string>("output", output, std::string("/primitive_marker_array"));
	pn.param<bool>("display_plane_box", display_plane_box, false);
	pn.param<double>("plane_r", plane_r, 1.0);
	pn.param<double>("plane_g", plane_g, 0.0);
	pn.param<double>("plane_b", plane_b, 0.0);
	
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	pub = n.advertise<visualization_msgs::MarkerArray>(output, 1);
	
	previous_n = 0;
    
    ros::spin();
    
    return 0;
}

