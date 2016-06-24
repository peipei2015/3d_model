#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    //
    // Fill in the CloudIn data, random
    //
    cloud_in->width    = 50;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
               << std::endl;                     
    for (size_t i = 0; i < cloud_in->points.size (); ++i) 
        std::cout << "    " <<cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
                cloud_in->points[i].z << std::endl;
                   
    //
    //out.x = in.x+0.7
    //
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        //cloud_out->points[i].x = cloud_in->points[i].x + 3.0f;
        cloud_out->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f) + 3.0f;
        cloud_out->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_out->points[i].z = cloud_out->points[i].y;
    }
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
            << std::endl;
    for (size_t i = 0; i < cloud_out->points.size (); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
                cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    
    //
    //results
    //
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
            icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //
    //visualize
    //
    pcl::visualization::PCLVisualizer viewer ("ICP example");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud_in, 5, 0, 230); //blue
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud_in, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_out, 150, 20, 20); // Red
    viewer.addPointCloud (cloud_out, transformed_cloud_color_handler, "cloud_filtered");

    viewer.addCoordinateSystem (1.0, 0);//viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.9, 0.9, 0.9, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_filtered");
    //viewer.setPosition(800, 400); // Setting visualiser window position
    
    
    while (!viewer.wasStopped ()) 
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
    return (0);
}
