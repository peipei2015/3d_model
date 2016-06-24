#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

int main (int argc, char** argv)
{
    //
    //visualization setting
    //
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer (argc, argv, "source data");
    int vp_1, vp_2;
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    
    //
    //initialize
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile<pcl::PointXYZ> ("milk_cartoon_all_small_clorox.pcd", *cloud);/*milk_cartoon_all_small_clorox.pcd*/
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
    
    //
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_f);
    std::cout << "PointCloud after filtering has: " << cloud_f->points.size ()  << " data points." << std::endl; //*
    
    //visualize
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_f, 200, 200, 200);
    p->addPointCloud (cloud_f, src_h, "src", vp_1);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "src");
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_r (cloud, 200, 200, 200);
    p->addPointCloud (cloud, src_r, "right", vp_2);
    //
    //plane segmentation
    //
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.02);    //1 cm
    seg.setInputCloud (cloud_f);
    seg.segment (*inliers, *coefficients);


    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_f);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_p);
    std::cout << "PointCloud representing the planar component: " << cloud_p->points.size () << " data points." << std::endl;
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_h (cloud_p, 150, 10, 10);
    p->addPointCloud (cloud_p, dst_h, "dst", vp_1);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dst");
          
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_c);
    std::cerr << "Model cloud_c: " << cloud_c->points.size () << std::endl;
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_h1 (cloud_c, 10, 150, 10);
    p->addPointCloud (cloud_c, dst_h1, "dst1", vp_1);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dst1");
    
   
    
    //
    //cloud_c, clustering
    // 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_c);
    std::vector<pcl::PointIndices> cluster_indices;
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_c);
    ec.extract (cluster_indices);
    
    int j = 0;   
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_c->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        
        //
        int r = rand()%256;
        int g = rand()%256;
        int b = rand()%256;
        char win[50];
        sprintf(win, "cluster_%d", j++);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_h3 (cloud_cluster, r, g, b);
        p->addPointCloud (cloud_cluster, dst_h3, win, vp_1);  
        
        
        //estimate normals
        //
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_cluster);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);
        
        //cylinder segment
        //
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;   //segment
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight(0.1);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud_cluster);
        seg.setInputNormals(cloud_normals);
        seg.segment(*inliers, *coefficients_plane);
        
        //extract cylinder
        //
        extract.setInputCloud (cloud_cluster);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter(*cloud_cylinder);
        
        sprintf(win, "cylinder_%d", j++);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_h4 (cloud_cylinder, 10, 10, 200);
        p->addPointCloud (cloud_cylinder, dst_h4, win, vp_1); 
    }
    std::cout<<"number of clusters : "<<j<<"\n";
    p->spin();
    
    return (0);
}
