#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width  = 10;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f)*2;
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f)*2;
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f)*2;
    }
  
    std::cerr << "Cloud before filtering: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.0, 0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 5, 0, 230);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_filtered, 230, 20, 20); // Red
    viewer.addPointCloud (cloud_filtered, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem (1.0, 0);//viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.9, 0.9, 0.9, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position
    
    
    while (!viewer.wasStopped ()) 
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
    return (0);
}
