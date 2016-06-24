#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud2;

void callback(const PointCloud2::ConstPtr& msg)
{
    cout<<"callback\n";
    PointCloud2::Ptr Points (new PointCloud2);
    *Points = *msg;
    pcl::io::savePCDFileASCII ("scene.pcd", *Points);
}

int main(int argc, char** argv)
{
    cout<<"running\n";
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud2>("/camera/depth/points", 1, callback);
    ros::spinOnce();
}

