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
// #include <map>

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "使用格式："
                  << "\n"
                  << "./to_octo path(pcd文件路径) [optional]resolution(octomap分辨率  default = 0.2)" << std::endl;
        return 1;
    }

    std::string PATH = argv[1];
    PATH += "/";
    
    double resolution = 0.2;
    if(argc == 3)
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

    std::string oct_path = PATH + "octo";
    if (access(oct_path.c_str(), 0) != 0)
    {
        mkdir(oct_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    octomap::OcTree* octree = new octomap::OcTree(resolution);
    int num_file = files.size();
    int pcd_count = 0;
    for(auto file : files)
    {
        std::string path = PATH + file;
        if(pcl::io::loadPCDFile(path.c_str(), *cloud) != -1)
        {
            if(cloud->width == 0 )
            {
                std::cerr<<path<<" is empty"<<std::endl;
                ++pcd_count;
                continue;
            }
            for(auto p : cloud->points)
            {
                octree->updateNode(octomap::point3d(p.x,p.y,p.z),true);
            }
            octree->updateInnerOccupancy();
            std::string name_index = std::to_string(pcd_count);
            std::string file_name = oct_path+"/octomap_0"+name_index+".bt";
            octree->writeBinary(file_name);
        }
        ++pcd_count;
        cloud->clear();
        octree->clear();
        std::cout << "to octomap " << std::fixed << std::setprecision(2)
                  << ((double)pcd_count) / ((double)num_file) * 100 << "%  " << path << std::endl;
       
    }
    
     time_end = clock();
     std::cout << "load time: " << (double)(time_end - time_start) / CLOCKS_PER_SEC << "S" << std::endl;
    return 0;
}
