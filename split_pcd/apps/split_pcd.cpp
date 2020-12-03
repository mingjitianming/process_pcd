/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <condition_variable>
#include <queue>
#include <thread>

#include <std_msgs/Bool.h>


#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <sys/stat.h>
#include <map>

int main(int argc, char **argv)
{
	

	if (argc < 3) {
		std::cerr<<"使用格式："<<"\n"
        <<"./split_pcd path(pcd路径) num(分割后pcd中点的数量,单位：万)"<<std::endl;
		return 1;
	}
		
		std::string PATH = argv[1];                             
	        PATH+="/";
                int num_point = atoi(argv[2])*10000;
              
		struct dirent *ptr;
   		DIR *dir;
	    dir=opendir(PATH.c_str());
	    std::vector<std::string> files; 

	    clock_t time_start,time_end;
	    time_start = clock();

	    while((ptr=readdir(dir))!=NULL)
	    {
	        //跳过'.'和'..'两个目录 和 文件夹
	        if(ptr->d_name[0] == '.'|| ptr->d_type==4)
	            continue; 
	     char* end = ptr->d_name;
             while((*end) != '.')
             {
               ++end;
              }
	      if(*(++end)=='p' && *(++end)=='c' && *(++end)=='d')
	        files.push_back(ptr->d_name);
	    }
	    closedir(dir);


	    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        std::string new_pcd_path = PATH+"split_pcd/";
        if(access(new_pcd_path.c_str(),0) != 0)
          {
            mkdir(new_pcd_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
           }
        int pcd_count = 0;
	int map_count = 0; 
	int processing = 0;
	int num_file = files.size();
	    for(std::string file : files)
	    {
	   
	    	std::string path = PATH+file;
	    	
	    	if(pcl::io::loadPCDFile(path.c_str(), *cloud)!= -1)
	    	{
				pcl::PointCloud<pcl::PointXYZI> pcd_split;
				

				//std::cout << "pcd_split inittial width & height " << pcd_split.width << ". " << pcd_split.height << std::endl;
				pcl::PointXYZI pt;

				 for(auto& item : cloud->points) {
			//	for (pcl::PointCloud<pcl::PointXYZI>::iterator item = cloud->begin(); item != cloud->end(); ++item) {

				  pt.x = (double)item.x;
				  pt.y = (double)item.y;
				  pt.z = (double)item.z;
				  pt.intensity = (double)item.intensity;    
				  pcd_split.push_back(pt);
				  pcd_count ++;
				  //1000000
				  if(pcd_count > num_point) {
				    std::string name_index = std::to_string(map_count);
				    std::string filename = "split_pcd_00" + name_index + ".pcd";
				    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcd_split));
				    map_ptr->header.frame_id = "map";
				  //  pcl::io::savePCDFileASCII(new_pcd_path+filename, *map_ptr);
				    pcl::io::savePCDFileBinary(new_pcd_path+filename, *map_ptr);
				    std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
				    std::cout << "pcd_split width & height " << pcd_split.width << "." << pcd_split.height << std::endl;
				    std::cout << "pcd_count " << pcd_count << std::endl;
				    pcd_split.clear();
				    pcd_split.width = 0;
				    pcd_split.height = 0;
				    pcd_count = 0;
				    map_count ++;
				  }
				}

				if(pcd_count > 0) {
				  std::string name_index = std::to_string(map_count);
				  std::string filename = "split_pcd_00" + name_index + ".pcd";
				  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcd_split));
				  map_ptr->header.frame_id = "map";
				  pcd_split.width = pcd_count;
				  pcd_split.height = 1;
				 // pcl::io::savePCDFileASCII(new_pcd_path+filename, *map_ptr);
       				  pcl::io::savePCDFileBinary(new_pcd_path+filename, *map_ptr);
				  std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
				  std::cout << "pcd_split width & height " << pcd_split.width << "." << pcd_split.height << std::endl;
				  std::cout << "pcd_count " << pcd_count << std::endl;
				}

			   std::cout << "Saved " << cloud->points.size() << " data points to " << "split_pcd" << "." << std::endl;
	    	}
	    	cloud->clear();
	    	++processing;
	    	std::cout<<"split pcd "<<std::fixed<<std::setprecision(2)<<((double)processing)/((double)num_file)*100<<"%  "<<path<<std::endl;
	    }

		time_end = clock();
		std::cout<<"load all time: "<<(double)(time_end - time_start) / CLOCKS_PER_SEC<<"S"<<std::endl;


	return 0;
}
