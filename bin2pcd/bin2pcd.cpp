#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
void PoindCloud_view(string filename_path, string window_name, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void readKittiPclBinData(std::string &in_file, std::string& out_file);
int main(int argc, char** argv)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	std::string bin_filename = "0000000000.bin";
	std::string pcd_filename = "0000000000.pcd";
	readKittiPclBinData(bin_filename, pcd_filename);

	return (0);
}

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

