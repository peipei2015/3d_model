#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;
//voxel grid --> parametric segmentation -->
int main (int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("table_scene_lms400.pcd", *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;



    //visualize
    pcl::visualization::PCLVisualizer viewer ("extract indices example");
    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler (cloud_filtered, 5, 0, 230); //blue
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud_filtered, color_handler, "filtered_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "filtered_cloud");
    viewer.addCoordinateSystem (1.0, 0);//viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.9, 0.9, 0.9, 0); // Setting background to a dark grey
    
    
    

    int seg_i = 1;
    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        std::cerr << "cloud_filtered = " << cloud_filtered->points.size ()<<std::endl;
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);      //the segment be stored in inliers
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);        
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        int r, g, b;
        r = rand () % (256);
        g = rand () % (256);
        b = rand () % (256);
        char winName[200];
        sprintf(winName, "cloud_p%d", seg_i++);
        //visualize
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler (cloud_p, r, g, b); // Red
        viewer.addPointCloud (cloud_p, color_handler, winName);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, winName);



        std::stringstream ss;
        ss << "table_scene_lms400_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);      //points except inliers
        cloud_filtered.swap (cloud_f);
        i++;
    }


    while (!viewer.wasStopped ()) 
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
    }
    


    return (0);
}
