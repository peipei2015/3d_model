#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //viewer->addCoordinateSystem (1.0, "global");
    viewer->initCameraParameters ();
    return (viewer);
}

int main(int argc, char** argv)
{
    int model_label = atoi(argv[1]);    //1 -- plane, 2 - sphere, 3 -- line, 4 -- cylinder, 5 -- cone
    
    //
    //visualization setting
    //
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer (argc, argv, "source data");
    int vp_1, vp_2;
    
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   
    
    //
    // initialize PointClouds
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width    = 500;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    
    if(model_label == 1)
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if( i % 2 == 0)
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else
                cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
        }
        
        std::vector<int> inliers;

        // created RandomSampleConsensus object and compute the appropriated model
        
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
        ransac.setDistanceThreshold (0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    }
    else if(model_label == 2)
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if (i % 5 == 0)
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else if(i % 2 == 0)
                cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x) - (cloud->points[i].y * cloud->points[i].y));
            else
                cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x) - (cloud->points[i].y * cloud->points[i].y));
        }
        
        std::vector<int> inliers;
        
        pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
        ransac.setDistanceThreshold (0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    }
    else if(model_label == 3)
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            if (i % 5 == 0)
            {
                cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            }    
            else
            {
                cloud->points[i].y = cloud->points[i].x;
                cloud->points[i].z = 1.0;
            }
        }
        
        std::vector<int> inliers;
        
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
        ransac.setDistanceThreshold (0.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
        
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    }
    else if(model_label == 4)
    {
        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            if (i % 5 == 0)
            {
                cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
                cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            }    
            else if(i%2 == 0)
            {
                cloud->points[i].y = sqrt( fabs(1.0 - fabs(1.0 - cloud->points[i].x)*fabs(1.0 - cloud->points[i].x)) );
            }
            else
            {
                cloud->points[i].y = -sqrt( fabs(1.0 - fabs(1.0 - cloud->points[i].x)*fabs(1.0 - cloud->points[i].x)) );
            }
        }
        
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (3.0);
        ne.compute (*cloud_normals);

        
        std::vector<int> inliers;
        
        pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr model_c(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> (cloud));
        model_c->normals_ = cloud_normals;
        
        
        
        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    }
    
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud, 255, 200, 200);
    p->addPointCloud (cloud, src_h, "src", vp_1);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "src");
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dst_h (final, 255, 200, 200);
    p->addPointCloud (final, dst_h, "dst", vp_2);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "dst");
    
    p->spin();
    
    return 0;
}
