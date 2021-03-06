#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointXYZ PointType2;

std::string model_filename_;

void showHelp (char *filename)
{
    std::cout << std::endl;
    std::cout << "Usage: " << filename << " model_filename.pcd" << std::endl << std::endl;
}

void parseCommandLine (int argc, char *argv[])
{
    //Show help
    if (pcl::console::find_switch (argc, argv, "-h"))
    {
        showHelp (argv[0]);
        exit (0);
    }

    //Model & scene filenames
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    if (filenames.size () != 1)
    {
        std::cout << "Filenames missing.\n";
        showHelp (argv[0]);
        exit (-1);
    }

    model_filename_ = argv[filenames[0]];
}

int main (int argc, char *argv[])
{
    parseCommandLine (argc, argv);

    pcl::PointCloud<PointType2>::Ptr model (new pcl::PointCloud<PointType2> ());
    
    //
    //  Load clouds
    //
    if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        showHelp (argv[0]);
        return (-1);
    }
   
    //
    //  Visualization
    //
    pcl::visualization::PCLVisualizer viewer ("model");
    viewer.addPointCloud (model, "model_cloud");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

    return (0);
}
