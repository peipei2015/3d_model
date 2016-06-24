#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
//write txt
#include <fstream>  
#include <vector>
#include <float.h>//isnan 
//pcl
#include <sensor_msgs/PointCloud2.h>  
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// time synchronize
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/callback_queue.h>
#include <boost/bind.hpp>
//seg
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

//global variable
pcl::visualization::CloudViewer viewer("Cloud Viewer");

void callback(const ImageConstPtr& image, const ImageConstPtr& image2, const PointCloud2::ConstPtr& Callback_Points);

int main(int argc, char **argv) 
{   	   
	//------------------------------初始化ros---------------------------------//
	ros::init(argc, argv, "objectdetect");
	ros::NodeHandle nh_;
   
    //三个话题的同步性，image_raw, depth/image, depth/points
    int global_subscriber_queue_size = 5;
    message_filters::Subscriber<sensor_msgs::Image> visual_sub_ (nh_, "/camera/rgb/image_raw", global_subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh_, "/camera/depth/image", global_subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_(nh_, "/camera/depth/points", global_subscriber_queue_size);
    
    message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10),  visual_sub_, depth_sub_, cloud_sub_);

    sync_.registerCallback(boost::bind(&callback, _1, _2, _3));
	ros::spin();
	
	//ros::Rate loop_rate(10); //frequency
	//while(ros::ok())
	//{
	//	ros::spinOnce();
	//}
	return 0;
}

void callback(const ImageConstPtr& image, const ImageConstPtr& image2, const PointCloud2::ConstPtr& Callback_Points)
{
    // Solve all of perception here...
    cout<<"synchronize...\n";
    //---------------获取彩色图像------------------//
    cv_bridge::CvImagePtr cv_ptr;
	try
	{
	   	cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	     	ROS_ERROR("cv_bridge exception: %s", e.what());
	      	return;
	}
	IplImage ipl_imgColor = cv_ptr->image;
	IplImage *imgColor= cvCloneImage(&ipl_imgColor);
	cvNamedWindow("color");
	cvShowImage("color", imgColor);
	cvWaitKey(100);
	/*//---------------获取深度图像------------------//
    try
    {
	    cv_ptr = cv_bridge::toCvCopy(image2, "");
    }
    catch (cv_bridge::Exception& e)
    {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
    }
    IplImage ipl_imgDepth = cv_ptr->image;
	IplImage *imgDepth = cvCloneImage(&ipl_imgDepth);
	cvNamedWindow("depth");
	cvShowImage("depth", imgDepth);
	cvWaitKey(100);
    cvReleaseImage(&imgDepth);*/
	
	//---------------获得点云数据------------------//
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> points_xyz;
    pcl::fromROSMsg (*Callback_Points, points_xyz);
    
	////添加rgb信息
    if(imgColor == NULL)
        return;
    PointCloud::Ptr points_xyzrgb (new PointCloud);
    int width = points_xyz.width;
    int height = points_xyz.height;
    //cout<<"points_xyz->width = "<<width<<"  points_xyz->height = "<<height<<"\n";
    //initialize points_xyzrgb
    points_xyzrgb->width = width;
    points_xyzrgb->height = height;
    points_xyzrgb->is_dense = false;
    points_xyzrgb->points.resize (points_xyzrgb->width * points_xyzrgb->height);
    
    for (int j = 0; j < width; j++)
    {
        for (int i = 0; i < height; i++)
        { 
            int b = (int)CV_IMAGE_ELEM(imgColor, uchar, i, j*3+0);
            int g = (int)CV_IMAGE_ELEM(imgColor, uchar, i, j*3+1);
            int r = (int)CV_IMAGE_ELEM(imgColor, uchar, i, j*3+2);
            
            float x = points_xyz(j,i).x;
            float y = points_xyz(j,i).y;
            float z = points_xyz(j,i).z;
            if(isnan(x) || isnan(y) || isnan(z))
            {
                //cout<<"out of range\n";
                continue;
            }
            //cout<<"{"<<i<<", "<<j<<"}\t"<<x<<"\t"<<y<<"\t"<<z<<"\n";             
            (*points_xyzrgb)(j,i).x = x;
            (*points_xyzrgb)(j,i).y = y;
            (*points_xyzrgb)(j,i).z = z;
            
            (*points_xyzrgb)(j,i).r = r;
            (*points_xyzrgb)(j,i).g = g;
            (*points_xyzrgb)(j,i).b = b;            
        }
    }
    
    //---------------- planar segmentation --------------------//
    //创建分割所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.1);

    seg.setInputCloud (points_xyz.makeShared ());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
            //<< coefficients->values[1] << " "
            //<< coefficients->values[2] << " " 
            //<< coefficients->values[3] << std::endl;

    //std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        int index = inliers->indices[i];
        (*points_xyzrgb).points[index].r = 255;
        (*points_xyzrgb).points[index].g = 0;
        (*points_xyzrgb).points[index].b = 0;
        
    }
   
    viewer.showCloud(points_xyzrgb);
    cvReleaseImage(&imgColor);
   
}
