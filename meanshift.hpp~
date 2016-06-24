#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <iostream>
using namespace pcl;
using namespace std;

#define GAUSS_KERNEL 1
#define EPANECHNIKOV_KERNEL 2
#define NONE_KERNEL 3
class MeanShift
{
public:
    size_t m_size;                              // number of points
    PointCloud<PointXYZ>::Ptr mp_pointcloud;    // pointcloud for kd-tree
    PointCloud<PointXYZ>::Ptr mp_local_mode;    // convergence points
    PointCloud<PointXYZ>::Ptr cluster_center;   // cluster centers, gravacity of mp_local_mode
    vector<PointIndices> cluster_indices;       // cluster labels
    
    float max_size;     // max size of cluster
    float min_size;     // min size od cluster
    bool set_max_size;  // label if set max_size
    bool set_min_size;  // label if set min_size
    
    float NEAREST_ZERO; // convergence radius
    float MERGE_RADIUS; // merge radius
    float C;            // gauss kernel parameters
    float R;            // window size
    int kernel;         // kernel
public:
    //initiailize
    MeanShift() : mp_local_mode(new pcl::PointCloud<PointXYZ>), cluster_center(new pcl::PointCloud<PointXYZ>)
    { 
        m_size = 0;
        R = 0.0f;
        set_max_size = false;
        set_min_size = false;
        NEAREST_ZERO = 0.01;
        C = 1.0f;
    }
    
    //release
    ~MeanShift()
    {
        cluster_indices.clear();
    }
    
    // set inpuit pointcloud
    bool setInputCloud(const PointCloud<PointXYZ>::Ptr pPntCloud)
    {
        m_size = pPntCloud->points.size();
        mp_local_mode->width = m_size;
        mp_local_mode->height = 1;
        mp_local_mode->points.resize(m_size);
       
        mp_pointcloud = pPntCloud;
        
        return true;
    }
    
    //set window size
    bool setKNNRadius(const float radius)
    {
        R = radius;
        return true;
    }
    
    //set convergence radius
    bool setConvergenceRadius(double r)
    {
        NEAREST_ZERO = r;
        return true;
    }
    
    //set merge radius 
    bool setMergeRadius(double r)
    {
        MERGE_RADIUS = r;
    }
    
    //set kernel
    bool setKernel(int k)
    {
        kernel = k;
        return true;
    }
    
    //set max_size
    bool setMaxSize(double size)
    {
        set_max_size = true;
        max_size = size;
        return true;
    }
    
    //set min_size
    bool setMinSize(double size)
    {
        set_min_size = true;
        min_size = size;
        return true;
    }
    
    //compute cluster centers
    bool computeClusterCenter()
    {
        for(size_t i=0;i<cluster_indices.size();i++)
        {
            if(set_max_size && cluster_indices.at(i).indices.size() > max_size)
            {
                for(size_t j=0;j<cluster_indices.at(i).indices.size();j++)
                {
                    cluster_indices.at(i).indices.at(j) = -1;
                }
            }
            else if(set_min_size && cluster_indices.at(i).indices.size() < min_size)
            {
                for(size_t j=0;j<cluster_indices.at(i).indices.size();j++)
                {
                    cluster_indices.at(i).indices.at(j) = -1;
                }
            }
            else
            {
                PointXYZ p(0, 0, 0);
                for(size_t j=0;j<cluster_indices.at(i).indices.size();j++)
                {
                    PointXYZ pt = mp_local_mode->points[cluster_indices.at(i).indices.at(j)];
                    p.x += pt.x;
                    p.y += pt.y;
                    p.z += pt.z;
                }
                p.x /= cluster_indices.at(i).indices.size();
                p.y /= cluster_indices.at(i).indices.size();
                p.z /= cluster_indices.at(i).indices.size();
                cluster_center->points.push_back(p);  
            }   
        }
        return true;
    }
    
    //gauss
    inline float guass(float x)
    {
        return C * 0.5 * exp(-0.5 * x);
    }
    
    inline float guass2(float x)
    {
        return C * sqrt(x) * exp(-0.5 * x);
    }
   
    //Epanechnikov, kernel
    inline float Epanechnikov(float x)
    {
        if(x >=0 && x <= 1)
            return (1-x);
        else
            return 0;
    }
    
    //compute diatance from p1 to p2
    float getPointDis(PointXYZ p1, PointXYZ p2)
    {
        float dis = sqrt( pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2) + pow((p1.z-p2.z), 2) );
        return dis;
    }
    
    //compute convergence points for each point
    bool execMeanShiftEachPoint(PointXYZ in_pnt, PointXYZ &out_pnt)
    {
        // Set up KDTree
        pcl::KdTreeFLANN<PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<PointXYZ>);

        tree->setInputCloud (mp_pointcloud);

        // Main Loop
        PointXYZ pnt = in_pnt;  //initial point

        while (1)
        {
            // Neighbors containers
            std::vector<int> k_indices;
            std::vector<float> k_distances;

            float sum_weigh = 0;
            PointXYZ pt(0, 0, 0);
            float dist_pnts = 0.0f;

            tree->radiusSearch (pnt, R, k_indices, k_distances);

            for (int i = 0, pnumber = k_indices.size(); i < pnumber; ++i)
            {
                //cout<<i<<"\n";
                size_t index = k_indices[i];
                PointXYZ &nbhd_pnt = mp_pointcloud->points[index];
                float sqr_dist = k_distances[i];
                float w;
                if(kernel == GAUSS_KERNEL)
                {
                    float gauss_param = sqr_dist / (R * R);
                    w = guass(gauss_param);  
                }
                else if(kernel == NONE_KERNEL || kernel == EPANECHNIKOV_KERNEL)
                {
                    w = 1; 
                }
                pt.x += nbhd_pnt.x * w;
                pt.y += nbhd_pnt.y * w;
                pt.z += nbhd_pnt.z * w;
                sum_weigh += w;
            }
            pt.x /= sum_weigh;
            pt.y /= sum_weigh;
            pt.z /= sum_weigh;

            dist_pnts = getPointDis(pt, pnt);
            
            pnt = pt;
            
            k_indices.clear();
            k_distances.clear();
            
            if (dist_pnts <= NEAREST_ZERO)
            {
                break;
            }        
        };
     
        out_pnt.x = pnt.x;
        out_pnt.y = pnt.y;
        out_pnt.z = pnt.z;

        return true;
    }

    //merge convergence labels
    bool mergeSameLocalModePoint()
    {
        //assert(v_localmode.points.size() == m_size);
        std::vector<bool> v_iscluster(m_size, false);   //记录是否已聚类

        for (size_t i = 0; i < m_size; ++i)
        {
            if(!v_iscluster[i])
            {
                PointIndices cluster;
                
                for (size_t j = i + 1; j < m_size; ++j)
                {
                    const PointXYZ & lmpnt1 = mp_local_mode->points[i];
                    const PointXYZ & lmpnt2 = mp_local_mode->points[j];
                    //PointXYZ  pnt1 = mp_pointcloud->points[i];
                    //PointXYZ  pnt2 = mp_pointcloud->points[j];
                    float dist = 0.0f;

                    dist = getPointDis(lmpnt1, lmpnt2);
                    if (dist <= MERGE_RADIUS)
                    {
                        //两个点可近似重合
                        if ( !v_iscluster[i] &&  !v_iscluster[j])   
                        {
                            //两个都没有搜索过
                            v_iscluster[i] = true;
                            cluster.indices.push_back(i);

                            v_iscluster[j] = true;
                            cluster.indices.push_back(j);
                        }
                        else if(v_iscluster[i] && !v_iscluster[j])
                        {
                            //i搜索过，j没有搜索过
                            v_iscluster[j] = true;
                            cluster.indices.push_back(j);
                        }
                        else
                        {
                            //都已经聚类，就不做处理
                        }
                    }  
                } 
                if(cluster.indices.size() > 0)
                    cluster_indices.push_back(cluster);
            }       
        } 
        
        computeClusterCenter();
        return true;
    }

    //process 
    bool process(vector<PointIndices> &mc_indices)
    {
        for (size_t i = 0; i < m_size; ++i)
        {
            const PointXYZ pnt = mp_pointcloud->points[i];
            execMeanShiftEachPoint(pnt, mp_local_mode->points[i]);
        }
        mergeSameLocalModePoint();
        mc_indices = cluster_indices;
        return true;
    }
};
