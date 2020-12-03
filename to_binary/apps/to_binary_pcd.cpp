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

/********************************************************************************************************************************


    将点云的数据格式转换为二进制形式


  ********************************************************************************************************************************/

#include <condition_variable>
#include <queue>
#include <thread>

#include <std_msgs/Bool.h>


#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <dirent.h>
#include <time.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/common/common.h>
#include <map>

int main(int argc, char **argv)
{
	

	if (argc < 2) {
		std::cerr<<"使用格式："<<"\n"
        <<"./to_binary path(pcd文件路径) "<<std::endl;
		return 1;
	}
		
		std::string PATH = argv[1];                                   
	        PATH+="/";
              
		struct dirent *ptr;
   		DIR *dir;
	    dir=opendir(PATH.c_str());
	    std::vector<std::string> files; 

	    clock_t time_start,time_end;
	    time_start = clock();

	    while((ptr=readdir(dir))!=NULL)
	    {
	        //跳过'.'和'..'两个目录
	        if(ptr->d_name[0] == '.'|| ptr->d_type == 4)
	            continue; 
	     char* end = ptr->d_name;
             
             while(end!= NULL && (*end) != '.')
             {
               ++end;
              }
	      if(*(++end)=='p' && *(++end)=='c' && *(++end)=='d')
	        files.push_back(ptr->d_name);
	    }
	    closedir(dir);
             
            std::string binary_pcd_path = PATH+"binary_pcd";
            if(access(binary_pcd_path.c_str(),0)!=0)
              { 
                 mkdir(binary_pcd_path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
               }

	    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	    
	    int num_file = files.size();
            int pcd_count=0;

	    for(std::string file : files)
	    {
	   
	    	std::string path = PATH+file;
	    	std::cout<<"processing "<<path<<std::endl; /////////////////////////////////////////
	    	if(pcl::io::loadPCDFile(path.c_str(), *cloud)!= -1)
	    	{
		   if(cloud->width==0)
		   {
		    std::cerr<<path<<"is null"<<std::endl;
		    ++ pcd_count;
		    continue;
		   }
		   
                   std::string name_index = std::to_string(pcd_count);
                   std::string file_name = binary_pcd_path+"/binary_pcd_0"+name_index+".pcd";
                   
		   pcl::io::savePCDFileBinary(file_name, *cloud);		
	    	}
	    	cloud->clear();
	    	++ pcd_count;
	    	std::cout<<"to binary "<<std::fixed<<std::setprecision(2)
	    	         <<((double)pcd_count)/((double)num_file)*100<<"%  "<<path<<std::endl;
	    }

      

		time_end = clock();
		std::cout<<"load all time: "<<(double)(time_end - time_start) / CLOCKS_PER_SEC<<"S"<<std::endl;


	return 0;
}
