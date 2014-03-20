#include <iostream>

#include "ros/ros.h"

#include "primitive_extraction/primitive.h"
#include "primitive_extraction/primitive_array.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>

ros::Publisher pub;
int previous_n;

void write_plane_marker(visualization_msgs::Marker& marker, const primitive_extraction::primitive& primitive)
{
    //marker.type = visualization_msgs::Marker::CUBE;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.pose.position.x = primitive.pose.position.x;
    marker.pose.position.y = primitive.pose.position.y;
    marker.pose.position.z = primitive.pose.position.z;
    marker.pose.orientation.x = primitive.pose.orientation.x;
    marker.pose.orientation.y = primitive.pose.orientation.y;
    marker.pose.orientation.z = primitive.pose.orientation.z;
    marker.pose.orientation.w = primitive.pose.orientation.w;
    marker.scale.x = 0.02;
    //marker.scale.x = 0.01;
    //marker.scale.y = primitive.params[0];
    //marker.scale.z = primitive.params[1];
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.points.resize(2*primitive.points.size());
    for (int i = 0; i < primitive.points.size()-1 ; ++i) {
        marker.points[2*i+2] = primitive.points[i];
        marker.points[2*i+3] = primitive.points[i+1];
    }
    marker.points[0] = primitive.points[primitive.points.size() - 1];
    marker.points[1] = primitive.points[0];
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
    visualization_msgs::MarkerArray markers;
    
    if (n >= previous_n) {
        markers.markers.resize(n);
    }
    else {
        markers.markers.resize(previous_n);
    }
    
    for (int i = 0; i < n; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "head_xtion_rgb_optical_frame";
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
        markers.markers[i].header.frame_id = "head_xtion_rgb_optical_frame";
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
	
    std::string input = "/primitives";
	ros::Subscriber sub = n.subscribe(input, 1, callback);
	
	std::string output = "/primitive_marker_array";
	pub = n.advertise<visualization_msgs::MarkerArray>(output, 1);
	
	previous_n = 0;
    
    ros::spin();
    
    return 0;
}

