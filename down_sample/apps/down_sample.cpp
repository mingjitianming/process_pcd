#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <dirent.h>
#include <iomanip>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc,char** argv)
{
    if (argc < 2)
    {
        std::cerr << "使用格式："
                  << "\n"
                  << "./down_sample path(pcd文件路径) [optional]resolution(default = 0.2)" << std::endl;
        return 1;
    }

    std::string PATH = argv[1];
    PATH += "/";

    double resolution = 0.2;
    if (argc == 3)
        resolution = atof(argv[2]);
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(PATH.c_str());
    std::vector<std::string> files;

    clock_t time_start, time_end;
    time_start = clock();

    while ((ptr = readdir(dir)) != NULL)
    {
        //跳过'.'和'..'两个目录
        if (ptr->d_name[0] == '.' || ptr->d_type == 4)
            continue;
        char *end = ptr->d_name;

        while (end != NULL && (*end) != '.')
        {
            ++end;
        }
        if (*(++end) == 'p' && *(++end) == 'c' && *(++end) == 'd')
            files.push_back(ptr->d_name);
    }
    closedir(dir);

    std::string down_sample_path = PATH + "down_sample";
    if (access(down_sample_path.c_str(), 0) != 0)
    {
        mkdir(down_sample_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }

    
    
    int num_file = files.size();
    int pcd_count = 0;

    for (auto file : files)
    {
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        std::string path = PATH + file;
        if(pcl::io::loadPCDFile(path.c_str(),*cloud) != -1)
        {
            if (cloud->width == 0)
            {
                std::cerr << path << " is empty" << std::endl;
                ++pcd_count;
                continue;
            }
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(resolution,resolution,resolution);
            sor.filter(*cloud_filtered);
            std::string file_name = down_sample_path+"/"+file;
         //   pcl::io::savePCDFileBinary(file_name,*cloud_filtered)
            pcl::PCDWriter writer;
           writer.writeBinary(file_name, *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity());
           //writer.writeASCII(file_name, *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity());
        }
        ++pcd_count;
        std::cout << "to octomap " << std::fixed << std::setprecision(2)
                  << ((double)pcd_count) / ((double)num_file) * 100 << "%  " << path << std::endl;
    }
    time_end = clock();
    std::cout << "load time: " << (double)(time_end - time_start) / CLOCKS_PER_SEC << "S" << std::endl;

    return 0;
}
