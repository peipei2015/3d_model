#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    //sor.setNegative (true);
    //sor.filter (*cloud_filtered);
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
    
    //visualize
    cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
    for(int i=0; i<cloud_filtered->points.size(); i++)
    {
        cloud_filtered->points[i].x += 2.5;
    }
    pcl::visualization::PCLVisualizer viewer ("outlier filter example");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 5, 0, 230); //blue
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (cloud_filtered, 150, 20, 20); // Red
    viewer.addPointCloud (cloud_filtered, transformed_cloud_color_handler, "cloud_filtered");

    viewer.addCoordinateSystem (1.0, 0);//viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.9, 0.9, 0.9, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered");
    //viewer.setPosition(800, 400); // Setting visualiser window position
    
    
    while (!viewer.wasStopped ()) 
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
      
    
    return (0);
}
