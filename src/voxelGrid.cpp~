#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
    /*pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    pcl::PCDWriter writer;
    writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    return (0);*/
    
    
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("table_scene_lms400.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //
    
    
    
    //visualize
    cloud_filtered->points.resize (cloud_filtered->width * cloud_filtered->height);
    for(int i=0; i<cloud_filtered->points.size(); i++)
    {
        cloud_filtered->points[i].x += 2.5;
    }
    pcl::visualization::PCLVisualizer viewer ("voxel grid example");
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
}
