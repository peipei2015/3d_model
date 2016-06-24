#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <iostream>
using namespace pcl;
using namespace std;

class MeanShift
{
public:
    size_t m_size;      //要处理点的个数
    PointCloud<PointXYZ>::Ptr mp_pointcloud;  //PCL形式的点云，主要是为了使用FLANN的KD树
    0PointCloud<PointXYZ>::Ptr mv_local_mode;
   
    float R;  //K近邻算法时对应的搜索半径

    /** \brief 对每个点执行MeanShift
    * \param[in]  in_pnt 输入的点
    * \param[out] out_pnt 输出的点
    */
    bool execMeanShiftEachPoint(PointXYZ in_pnt, PointXYZ &out_pnt);

    /** \brief 将具有相同局部模式的点归为一类
    * \param[in] v_localmode 各个点对应的局部模式
    * \param[out] vv_pnt 归并后的点
    */
    bool mergeSameLocalModePoint(const PointCloud<PointXYZ>::Ptr v_localmode, vector< vector<PointXYZ> > vv_pnt);

    inline float gauss(float x)
    {
        return C * sqrt(x) * exp(-0.5 * x);
    }
    float getPointDis(PointXYZ p1, PointXYZ p2)
    {
        float dis = sqrt( pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2) + pow((p1.z-p2.z), 2) );
        return dis;
    }
    
public:
    float NEAREST_ZERO;
    float C;  //!高斯核函数中的常数
    vector< vector<PointXYZ> > vv_pnt;    //聚类结果

    MeanShift() : m_size(0), R(0.0f), mv_local_mode(new pcl::PointCloud<PointXYZ>)
    { 
        NEAREST_ZERO = 0.01;
        C = 2.0f;
    }

    /** \brief 设置输入点云
    * \param[in]  pPntCloud 输入点云
    */
    bool setInputCloud(const PointCloud<PointXYZ>::Ptr pPntCloud)
    {
        m_size = pPntCloud->points.size();
        mv_local_mode->width = m_size;
        mv_local_mode->height = 1;
        mv_local_mode->points.resize(m_size);
       
        mp_pointcloud = pPntCloud;
        
        return true;
    }

    /** \brief 设置K近邻算法的搜索半径
    * \param[in]  radius 半径
    */
    bool setKNNRadius(const float radius)
    {
        R = radius;
        return true;
    }

    /** \brief 执行MeanShift聚类算法	   */
    bool process()
    {
        //cout<<"m_size = "<<m_size<<"\n";
        for (size_t i = 0; i < m_size; ++i)
        {
            const PointXYZ pnt = mp_pointcloud->points[i];
            execMeanShiftEachPoint(pnt, mv_local_mode->points[i]);
        }
        //cout<<"done\n";
        mergeSameLocalModePoint(mv_local_mode, vv_pnt);

        return true;
    }
};

bool MeanShift::execMeanShiftEachPoint(PointXYZ in_pnt, PointXYZ &out_pnt)
{
    // Set up KDTree
    pcl::KdTreeFLANN<PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<PointXYZ>);

    tree->setInputCloud (mp_pointcloud);

    // Main Loop
    PointXYZ pnt = in_pnt;

    while (1)
    {
        // Neighbors containers
        std::vector<int> k_indices;
        std::vector<float> k_distances;

        float sum_weigh = 0;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float dist_pnts = 0.0f;

        tree->radiusSearch (pnt, R, k_indices, k_distances);

        for (int i = 0, pnumber = k_indices.size(); i < pnumber; ++i)
        {
            //cout<<i<<"\n";
            size_t index = k_indices[i];
            PointXYZ &nbhd_pnt = mp_pointcloud->points[index];
            float sqr_dist = k_distances[i];
            float gauss_param = sqr_dist / (R * R);
            float w = gauss(gauss_param);

            x += nbhd_pnt.x * w;
            y += nbhd_pnt.y * w;
            z += nbhd_pnt.z * w;
            sum_weigh += w;
        }
        x = x / sum_weigh;
        y = y / sum_weigh;
        z = z / sum_weigh;

        float diff_x = x - pnt.x, diff_y = y - pnt.y, diff_z = z - pnt.z;

        dist_pnts = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
        //cout<<"p\n";
        
        pnt.x = x;
        pnt.y = y;
        pnt.z = z;
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

bool MeanShift::mergeSameLocalModePoint(const PointCloud<PointXYZ>::Ptr v_localmode, vector< vector<PointXYZ> > vv_pnt)
{
    //assert(v_localmode.points.size() == m_size);
    std::vector<bool> v_iscluster(m_size, false);   //记录是否已聚类

    for (size_t i = 0; i < m_size; ++i)
    {
        if(!v_iscluster[i])
        {
            vector<PointXYZ> v_pnt;
            
            for (size_t j = i + 1; j < m_size; ++j)
            {
                const PointXYZ & lmpnt1 = v_localmode->points[i];
                const PointXYZ & lmpnt2 = v_localmode->points[j];
                PointXYZ  pnt1 = mp_pointcloud->points[i];
                PointXYZ  pnt2 = mp_pointcloud->points[j];
                float dist = 0.0f;

                dist = getPointDis(lmpnt1, lmpnt2);
                if (dist <= NEAREST_ZERO)
                {
                    //两个点可近似重合
                    if ( !v_iscluster[i] &&  !v_iscluster[j])   
                    {
                        //两个都没有搜索过
                        v_iscluster[i] = true;
                        v_pnt.push_back(pnt1);

                        v_iscluster[j] = true;
                        v_pnt.push_back(pnt2);
                    }
                    else if(v_iscluster[i] && !v_iscluster[j])
                    {
                        //i搜索过，j没有搜索过
                        v_iscluster[j] = true;
                        v_pnt.push_back(pnt2);
                    }
                    else
                    {
                        //都已经聚类，就不做处理
                    }
                }  
            } 
            
            vv_pnt.push_back(v_pnt);
            v_pnt.clear();
        }       
    } 
    cout<<"vv_pnt.size() = "<<vv_pnt.size()<<"\n";
    return true;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld (new pcl::PointCloud<pcl::PointXYZ>);		
    int val = 10;
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
                basic_point.x = (float)(rand() % val) / 10.0f + 2.5;    
                basic_point.y = (float)(rand() % val) / 10.0f + 2.5;    
                basic_point.z = (float)(rand() % val) / 10.0f + 2.5;    
                cld->points.push_back(basic_point);    
            }  
        }  
    }  

    for (float x = 0; x < cube_len; x += 0.1)  
    {  
        for (float y = 0; y < cube_len; y += 0.1)  
        {  
            for (float z = 0; z < cube_len; z += 0.1)  
            {  
                pcl::PointXYZ basic_point;    

                basic_point.x = (float)(rand() % val) / 10.0f - 2.5;    
                basic_point.y = (float)(rand() % val) / 10.0f - 2.5;    
                basic_point.z = (float)(rand() % val) / 10.0f - 2.5;    
                cld->points.push_back(basic_point);    
            }  
        }  
    }  

    cld->width = (int)cld->points.size();    
    cld->height = 1;  
    //io::savePCDFileASCII("manual_build.pcd", *cld);

    MeanShift ms;
    ms.setInputCloud(cld);
    ms.setKNNRadius(0.7);
    ms.process(); 
    
    
    //
    //visualize
    //
    pcl::visualization::PCLVisualizer viewer ("meanshift example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cld, 5, 0, 230); //blue
    viewer.addPointCloud (cld, source_cloud_color_handler, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");

    pcl::visualization::PCLVisualizer viewer2 ("cluster example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (ms.mv_local_mode, 5, 230, 230); //blue
    viewer2.addPointCloud (ms.mv_local_mode, source_cloud_color_handler2, "original_cloud2");
    viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
    
    viewer2.addPointCloud()
    
    while (!viewer.wasStopped ()) 
    {
        viewer.spinOnce ();
        viewer2.spinOnce ();
    }
    
    
    system("pause");
    return 0;
}
