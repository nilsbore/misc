#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include "primitive_extractor.h"
#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"

#include "primitive_extraction/primitive.h"
#include "primitive_extraction/primitive_array.h"

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
    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "extraction");
	ros::NodeHandle n;
	
    std::string camera_topic = "/head_xtion";
	ros::Subscriber sub = n.subscribe(camera_topic + "/depth_registered/points", 1, callback);
    
    ros::spin();
    
    return 0;
}

