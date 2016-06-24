#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
//This tutorial demonstrates how to create a range image from a point cloud and a given sensor position. 
int main (int argc, char** argv) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud( new pcl::PointCloud<pcl::PointXYZ>);

    // Generate the data
    for (float y=-0.5f; y<=0.5f; y+=0.01f) 
    {
        for (float z=-0.5f; z<=0.5f; z+=0.01f) 
        {
            pcl::PointXYZ point;
            point.x = 2.0f - y;
            point.y = y;
            point.z = z;
            pointCloud->points.push_back(point);
        }
    }
    pointCloud->width = (uint32_t) pointCloud->points.size();
    pointCloud->height = 1;

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(*pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    std::cout << rangeImage << "\n";
    
    //visualization
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (rangeImage);
    
    pcl::visualization::PCLVisualizer viewer ("cloud example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (pointCloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZ> (pointCloud, keypoints_color_handler, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }
}
