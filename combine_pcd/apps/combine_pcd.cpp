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


    使用地图片段的质心以及和机器人的角度选取地图片段


  ********************************************************************************************************************************/

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
//#include <pcl/common/common.h>
#include <map>

int main(int argc, char **argv)
{
	

	if (argc < 2) {
		std::cerr<<"使用格式："<<"\n"
        <<"./combine_pcd path(pcd文件路径) "<<std::endl;
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
	        if(ptr->d_name[0] == '.'|| ptr->d_type==4)
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


	    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            
            int processing = 0;
            int num_file = files.size();
	    for(std::string file : files)
	    {
	   
	    	std::string path = PATH+file;
	    	
	    	if(pcl::io::loadPCDFile(path.c_str(), *cloud)!= -1)
	    	{
				if(out_cloud->width==0)
				{
				  *out_cloud=*cloud;
				}
				else
				{
                                   *out_cloud+=*cloud;
				}
	    	  
	    	}
	    	cloud->clear();
	    	++processing;
	    	std::cout<<"combine pcd "<<std::fixed<<std::setprecision(2)
	    	         <<((double)processing)/((double)num_file)*100<<"%  "<<path<<std::endl;
	    }
	    

    //  pcl::io::savePCDFileASCII (PATH+"combined_pcd.pcd", *out_cloud);
           pcl::io::savePCDFileBinary (PATH+"combined_pcd.pcd", *out_cloud);
		time_end = clock();
		std::cout<<"load all time: "<<(double)(time_end - time_start) / CLOCKS_PER_SEC<<"S"<<std::endl;


	return 0;
}
