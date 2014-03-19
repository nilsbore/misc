#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include "primitive_extractor.h"
#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"

#include "primitive_extraction/primitive.h"
#include "primitive_extraction/primitive_array.h"

#include <Eigen/Dense>

ros::Publisher pub;

void write_plane_msg(primitive_extraction::primitive& msg, const Eigen::VectorXd& data, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points)
{
    msg.type = "plane";
    msg.pose.position.x = data(6);
    msg.pose.position.y = data(7);
    msg.pose.position.z = data(8);
    msg.pose.orientation.x = data(9);
    msg.pose.orientation.y = data(10);
    msg.pose.orientation.z = data(11);
    msg.pose.orientation.w = data(12);
    msg.params.resize(2);
    msg.params[0] = data(4);
    msg.params[1] = data(5);
    msg.points.resize(points.size());
    for (int i = 0; i < points.size(); ++i) {
        msg.points[i].x = points[i](0);
        msg.points[i].y = points[i](1);
        msg.points[i].z = points[i](2);
    }
}

void write_cylinder_msg(primitive_extraction::primitive& msg, const Eigen::VectorXd& data)
{
    msg.type = "cylinder";
    msg.pose.position.x = data(3);
    msg.pose.position.y = data(4);
    msg.pose.position.z = data(5);
    Eigen::Vector3d x = data.segment<3>(0);
    Eigen::Vector3d y(-x(1), x(0), 0);
    y.normalize();
    Eigen::Vector3d z = x.cross(y);
    z.normalize();
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    Eigen::Quaterniond quat(R);
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.params.resize(2);
    msg.params[0] = data(6); // radius
    msg.params[1] = data(7); // height
}

void write_sphere_msg(primitive_extraction::primitive& msg, const Eigen::VectorXd& data)
{
    msg.type = "sphere";
    msg.pose.position.x = data(0);
    msg.pose.position.y = data(1);
    msg.pose.position.z = data(2);
    Eigen::Quaterniond quat;
    quat.setIdentity();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.params.resize(1);
    msg.params[0] = data(3); // radius
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    
    primitive_params params;
    params.octree_res = 0.04;
    params.normal_neigbourhood = 0.015;
    params.inlier_threshold = 0.015;
    params.angle_threshold = 0.4;
    params.add_threshold = 0.01;
    params.min_shape = 3000;
    params.inlier_min = params.min_shape;
    params.connectedness_res = 0.01;
    params.distance_threshold = 2.0;
    
    std::vector<base_primitive*> primitives;
    primitives.push_back(new plane_primitive());
    primitives.push_back(new sphere_primitive());
    primitives.push_back(new cylinder_primitive());

    primitive_extractor extractor(cloud, primitives, params, NULL);
    std::vector<base_primitive*> extracted;
    extractor.extract(extracted);

    std::cout << "The algorithm has finished..." << std::endl;
    
    primitive_extraction::primitive_array msg_array;
    msg_array.primitives.resize(extracted.size());
    
    for (int i = 0; i < extracted.size(); ++i) {
        primitive_extraction::primitive msg;
        Eigen::VectorXd data;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
        extracted[i]->shape_data(data);
        switch (extracted[i]->get_shape()) {
        case base_primitive::PLANE: 
            extracted[i]->shape_points(points);
            write_plane_msg(msg, data, points);
            break;
        case base_primitive::CYLINDER:
            write_cylinder_msg(msg, data);
            break;
        case base_primitive::SPHERE:
            write_sphere_msg(msg, data);
            break;
        }
        msg_array.primitives[i] = msg;
    }
    
    pub.publish(msg_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extraction");
	ros::NodeHandle n;
	
    std::string camera_topic = "/head_xtion";
	ros::Subscriber sub = n.subscribe(camera_topic + "/depth_registered/points", 1, callback);
	
	std::string output = "/primitives";
	pub = n.advertise<primitive_extraction::primitive_array>(output, 1);
    
    ros::spin();
    
    return 0;
}

