#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
using namespace pcl;
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    cloud->width = 100;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    // Generate the data
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1.0;
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree1);
    ne.setRadiusSearch (3.0);
    // Compute the features
    ne.compute (*normals);

    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);

    // Compute the features
    fpfh.compute (*fpfhs);

    // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
    return 0;
}
