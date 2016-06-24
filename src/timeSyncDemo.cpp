//2016年04月19日 星期二 09时05分42秒 
//信息获取时间一致
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <ros/callback_queue.h>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
//记录时间
#include<time.h> 
time_t start, end;  
using namespace sensor_msgs;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

void callback(const ImageConstPtr& image, const ImageConstPtr& image2, const PointCloud2::ConstPtr& Callback_Points)
{
    // compute time consumption
    static int label = 1;
    if(label > 1001)
        return;
    if(label == 1)
    {
        start = time(NULL);
         cout<<"update start: "<<start<<"\n";
    }                                                                                                     
    if(label == 1000)
    {
        end = time(NULL);
        cout<<"update end: "<<end<<"\n"; 
    }
    if(label > 1000)
    {
        cout<<"clocks_per_sec = "<<CLOCKS_PER_SEC<<"\n";
        double d = (double)(end-start)/1000.0;
        cout<<d<<" fps = "<<1/d<<" frames/s\n";
    }
    label++;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;
    cout<<"build node done\n";
   
    int global_subscriber_queue_size = 1;
    
    message_filters::Subscriber<sensor_msgs::Image> visual_sub_ (nh, "/camera/rgb/image_raw", global_subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh, "/camera/depth/image", global_subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_(nh, "/camera/depth/points", global_subscriber_queue_size);
    
    message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10),  visual_sub_, depth_sub_, cloud_sub_);

    cout<<"initialize done\n";
    sync_.registerCallback(boost::bind(&callback, _1, _2, _3));
  
    ros::spin();

    return 0;
}

