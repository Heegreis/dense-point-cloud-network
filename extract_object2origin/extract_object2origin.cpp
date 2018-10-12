#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include "tracklets.h"


#include <string>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int get_file_count(char *dirname)
{
	DIR *dir;
	struct dirent *ptr;
	int total_count = 0;
	char path[PATH_MAX] = {0};
	dir = opendir(dirname);
	if (dir == NULL)
	{
		printf("%s: open dir: %s failed.\n", __func__, path);
		exit(1);
	}
	while ((ptr = readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
			continue;
		snprintf(path, (size_t)PATH_MAX, "%s/%s", dirname, ptr->d_name);
		if (ptr->d_type == DT_DIR)
		{
			total_count++;
			//printf("%s: Entering into dir: %s.\n", __func__,path);
			//printf("%s: Is dir, and count++, count: %ld\n", __func__, total_count);
			total_count += get_file_count(path);
			memset(path, 0, sizeof(path));
		}
		else

		{
			//printf("%s: count: %ld, file: %s.\n", __func__,total_count,path);
			total_count++;
		}
	}
	closedir(dir);
	//printf("%s: statistics total files count :%ld.\n", __func__,total_count);
	return total_count;
}

int main()
{
	string base_dir = "/data/dataset/kitti/raw_data/City0005";
	string data_date = "2011_09_26";
	string data_num = "0005";
	string sync_dir = data_date + "_drive_" + data_num + "_sync";
	string pcd_dir = base_dir + "/" + data_date + "_drive_" + data_num + "_pcd";
	string tracklets_dir = base_dir + "/" + data_date + "_drive_" + data_num + "_tracklets/" + data_date + "/" + sync_dir;

	//creat the save dir
	std::string save_dir = base_dir + "/" + data_date + "_drive_" + data_num + "_pcd_object_origin";
	char *save_dir_char = (char*)save_dir.c_str();
	if(access(save_dir_char, 0) != 0)
		int result = mkdir(save_dir_char, S_IRWXU|S_IRWXG|S_IRWXO);
	else
		std::cout << "the dir for save pcd file is existed." << std::endl;

	//build tracklets data
	Tracklets *tracklets = new Tracklets();
	if (!tracklets->loadFromFile(tracklets_dir + "/tracklet_labels.xml"))
		std::cerr << "load trackletFiles error" << std::endl;
	else
		std::cerr << "load trackletFiles sucess" << std::endl;
	int nTracklet = tracklets->numberOfTracklets();
	vector<Tracklets::tTracklet*> trackletsVec;
	for(int i = 0; i < nTracklet; i++)
	{
		Tracklets::tTracklet *tracklet;
		tracklet = tracklets->getTracklet(i);
		trackletsVec.push_back(tracklet);
	}

	//count the number of files
	char *pcd_dir_char = (char*)pcd_dir.c_str();
	int total_frame = 0;
	total_frame = get_file_count(pcd_dir_char);
	
	for(int frame = 0; frame < total_frame; frame++)
	{
		char filenum_str[14];
		sprintf(filenum_str, "%010d", frame);
		std::string pcd_filename = filenum_str + std::string(".pcd");

		//new in & out point cloud object and load cloud_in data
		PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
		if(pcl::io::loadPCDFile(pcd_dir + "/" + pcd_filename, *cloud_in) != 0)
			std::cout << "\nLoad cloud_in error\n" << std::endl;

		for(int tracklet_id = 0; tracklet_id < nTracklet; tracklet_id++)
		{
			if (frame >= trackletsVec[tracklet_id]->first_frame && frame <= trackletsVec[tracklet_id]->lastFrame()) {
				int first_frame = trackletsVec[tracklet_id]->first_frame;
				int poses_frame = frame - first_frame;
				
				float h = trackletsVec[tracklet_id]->h;
				float w = trackletsVec[tracklet_id]->w;
				float l = trackletsVec[tracklet_id]->l;

				double tx = trackletsVec[tracklet_id]->poses[poses_frame].tx;
				double ty = trackletsVec[tracklet_id]->poses[poses_frame].ty;
				double tz = trackletsVec[tracklet_id]->poses[poses_frame].tz;

				double rx = trackletsVec[tracklet_id]->poses[poses_frame].rx;
				double ry = trackletsVec[tracklet_id]->poses[poses_frame].ry;
				double rz = trackletsVec[tracklet_id]->poses[poses_frame].rz;

				double minX = 0 - (l / 2);
				double minY = 0 - (w / 2);
				double minZ = 0;

				double maxX = (l / 2);
				double maxY = (w / 2);
				double maxZ = h;

				PointCloudT::Ptr cloud_out(new PointCloudT);  // Out
				pcl::CropBox<PointT> filter;
				filter.setInputCloud(cloud_in);
				filter.setMin(Eigen::Vector4f((float)minX, (float)minY, (float)minZ, 1.0));
				filter.setMax(Eigen::Vector4f((float)maxX, (float)maxY, (float)maxZ, 1.0));
				filter.setRotation(Eigen::Vector3f((float)rx, (float)ry, (float)rz));//rotation
				filter.setTranslation(Eigen::Vector3f((float)tx, (float)ty, (float)tz));
				
				filter.filter(*cloud_out);

                //set the object to origin of coordinate
                pcl::transformPointCloud(*cloud_out, *cloud_out, pcl::getTransformation(-(float)tx, -(float)ty, -(float)tz, 0, 0, 0));
                pcl::transformPointCloud(*cloud_out, *cloud_out, pcl::getTransformation(0, 0, 0, -(float)rx, -(float)ry, -(float)rz));

				PointCloudT cloud_B;
				cloud_B.points = cloud_out.get()->points;
				cloud_B.height = cloud_out.get()->height;
				cloud_B.width = cloud_out.get()->width;

				char id_str[10];
				sprintf(id_str, "%d", tracklet_id);
				char poses_frame_str[10];
				sprintf(poses_frame_str, "%d", poses_frame);
				pcl::io::savePCDFileASCII(save_dir + "/" + filenum_str + "_" + trackletsVec[tracklet_id]->objectType + "_" + id_str + "_" + poses_frame_str + "_origin.pcd", cloud_B);
				std::cout << save_dir + "/" + filenum_str + "_" + trackletsVec[tracklet_id]->objectType + "_" + id_str + "_" + poses_frame_str + "_origin.pcd created success" << std::endl;
			}
		}
	}
	return 0;
}
