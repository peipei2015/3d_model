#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
#include "meanshift.hpp"
using namespace pcl;
using namespace std;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld (new pcl::PointCloud<pcl::PointXYZ>);		
    int val = 100;
    float val2 = 100.0f;
    //构造立方体  
    float cube_len = 1;  
    srand(time(NULL));
    for (float x = 0; x < cube_len; x += 0.1)  
    {  
        for (float y = 0; y < cube_len; y += 0.1)  
        {  
            for (float z = 0; z < cube_len; z += 0.1)  
            {  
                pcl::PointXYZ basic_point;    

                //沿着向量(2.5, 2.5, 2.5)平移  
                basic_point.x = (float)(rand() % val) / val2 + 1.0;    
                basic_point.y = (float)(rand() % val) / val2 + 0.0;    
                basic_point.z = (float)(rand() % val) / val2 + 0.0;    
                cld->points.push_back(basic_point);   
                
                basic_point.x = (float)(rand() % val) / val2 + 0.0;    
                basic_point.y = (float)(rand() % val) / val2 + 1.0;    
                basic_point.z = (float)(rand() % val) / val2 + 0.0;    
                cld->points.push_back(basic_point);
                
                basic_point.x = (float)(rand() % val) / val2 + 0.0;    
                basic_point.y = (float)(rand() % val) / val2 + 0.0;    
                basic_point.z = (float)(rand() % val) / val2 + 1.0;    
                cld->points.push_back(basic_point);
            }  
        }  
    }  

    cld->width = (int)cld->points.size();    
    cld->height = 1;  
    //io::savePCDFileASCII("manual_build.pcd", *cld);

    MeanShift ms;
    ms.setInputCloud(cld);
    ms.setKNNRadius(0.3);
    ms.setConvergenceRadius(0.01);
    ms.setMergeRadius(1.0);
    ms.setKernel(1);
    ms.setMinSize(100);
    vector<PointIndices> clusters;
    ms.process(clusters); 
    
    //
    //visualize
    //
    pcl::visualization::PCLVisualizer viewer ("meanshift example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cld, 230, 230, 230); 
    viewer.addPointCloud (cld, source_cloud_color_handler, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler (ms.mp_local_mode, 0, 230, 0); 
    viewer.addPointCloud (ms.mp_local_mode, color_handler, "mode_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "mode_cloud");

    pcl::visualization::PCLVisualizer viewer2 ("cluster example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (ms.cluster_center, 230, 10, 10);
    viewer2.addPointCloud (ms.cluster_center, source_cloud_color_handler2, "original_cloud2");
    viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "original_cloud2");
    
    cout<<"clusters = "<<clusters.size()<<"\n";
    for(size_t i=0;i<clusters.size();i++)
    {
        cout<<clusters.at(i).indices.size()<<"\t";
    }
    cout<<"\n";
    
    for(size_t i=0;i<clusters.size();i++)
    {
        PointCloud<PointXYZ>::Ptr cluster(new PointCloud<PointXYZ>), cluster2(new PointCloud<PointXYZ>);
        for(size_t j=0;j<clusters.at(i).indices.size();j++)
        {
            if(clusters.at(i).indices.at(j) == -1)
                continue;
            PointXYZ pt = cld->points[clusters.at(i).indices.at(j)];
            cluster->points.push_back(pt);
        }

        char name[50];
        sprintf(name, "cluster_%d", i);
        int r = rand()%256;
        int g = rand()%256;
        int b = rand()%256;
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_handler (cluster, r, g, b); 
        viewer2.addPointCloud(cluster, cluster_handler, name);
        viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    }
    
    while (!viewer.wasStopped ()) 
    {
        viewer.spinOnce ();
        viewer2.spinOnce ();
    }
    
    
    system("pause");
    return 0;
}
