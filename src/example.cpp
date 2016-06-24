#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>

#include <iostream>
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

//typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

char *window_name = "Input";
pcl::visualization::CloudViewer viewer("Cloud Viewer");
IplImage *image;

void pointsCb(const PointCloud::ConstPtr& Callback_Points)
{
    PointCloud::Ptr Points (new PointCloud);
    *Points = *Callback_Points;
 
    if(image == NULL)
        return;
    //添加rgb信息
    cout<<"Points->width = "<<Points->width<<"  Points->height = "<<Points->height<<"\n";
    for (int j = 0; j < Points->width; j++)
    {
        for (int i = 0; i < Points->height; i++)
        { 
            //float x = (float)(*input_cloud)(j,i).x;
            //float y = (float)(*input_cloud)(j,i).y;
            //float z = (float)(*input_cloud)(j,i).z;
            int b = (int)CV_IMAGE_ELEM(image, uchar, i, j*3+0);
            int g = (int)CV_IMAGE_ELEM(image, uchar, i, j*3+1);
            int r = (int)CV_IMAGE_ELEM(image, uchar, i, j*3+2);
            
            (*Points)(j,i).r = r;
            (*Points)(j,i).g = g;
            (*Points)(j,i).b = b;
        }
    }
    viewer.showCloud(Points);
}
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    IplImage ipl_img = cv_ptr->image;
    image= cvCloneImage(&ipl_img);
    cvNamedWindow(window_name);
    cvShowImage(window_name, image);
    cvWaitKey(100);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "example");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_(nh);
    ros::Subscriber points_sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, &pointsCb);
    image_transport::Subscriber imageColor_sub = it_.subscribe("/camera/rgb/image_raw", 1, &imageCb);
    //ros::Subscriber imageDepth_sub = it_.subscribe("/camera/depth/image", 1, &imageCb);
    
    ros::spin();
}

