#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

void readKittiPclBinData(std::string &in_file, std::string& out_file)
{
	// load point cloud
	std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
	if (!input.good()) {
		std::cerr << "Could not read file: " << in_file << std::endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, std::ios::beg);

	pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);

	int i;
	for (i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
	//    g_cloud_pub.publish( points );

	std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
	pcl::PCDWriter writer;
	string filename_string = "TEST";

	// Save DoN features
	writer.write< pcl::PointXYZI >(out_file, *points, false);
}

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

int main(int argc, char** argv)
{
	std::string base_dir = "/data/dataset/kitti/raw_data/City0005";
	std::string data_date = "2011_09_26";
	std::string data_num = "0005";
	std::string sync_dir = data_date + "_drive_" + data_num + "_sync";
	std::string vp_dir = base_dir + "/" + sync_dir + "/" + data_date + "/" + sync_dir + "/velodyne_points/data";

	//count the number of files
	char *vp_dir_char = (char*)vp_dir.c_str();
	int total_count = 0;
	total_count = get_file_count(vp_dir_char);
	printf("total_count:%d\n", total_count);

	//creat the save dir
	std::string save_dir = base_dir + "/" + data_date + "_drive_" + data_num + "_pcd";
	char *save_dir_char = (char*)save_dir.c_str();
	if(access(save_dir_char, 0) != 0)
		int result = mkdir(save_dir_char, S_IRWXU|S_IRWXG|S_IRWXO);
	else
		std::cout << "the dir for save pcd file is existed." << std::endl;

	for(int i = 0; i < total_count; i++)
	{
		char filenum_str[14];
		sprintf(filenum_str, "%010d", i);

		std::string bin_filename = vp_dir + "/" + filenum_str + ".bin";
		std::string pcd_filename = save_dir + "/" + filenum_str + ".pcd";
		/*std::string bin_filename = "0000000000.bin";
		std::string pcd_filename = "0000000000.pcd";*/
		readKittiPclBinData(bin_filename, pcd_filename);
		std::cout << filenum_str << ".pcd created sucess" << std::endl;
	}

	return (0);
}
