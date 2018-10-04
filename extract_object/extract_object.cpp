#include <pcl/io/pcd_io.h>
#include<pcl/PCLPointCloud2.h>
#include<iostream>
#include<string>
#include <pcl/filters/crop_box.h>
#include "tracklets.h"

using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main()
{
	PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_out(new PointCloudT);  // Out
	pcl::io::loadPCDFile("0000000000.pcd", *cloud_in);
	std::cout << "\nLoad cloud_in success\n" << std::endl;
	
	Tracklets *tracklets = new Tracklets();
	if (!tracklets->loadFromFile("tracklet_labels.xml"))
		std::cerr << "load trackletFiles error" << std::endl;
	else
		std::cerr << "load trackletFiles sucess" << std::endl;
	int nTrack = tracklets->numberOfTracklets();
	std::cerr << nTrack << std::endl;

	vector<Tracklets::tTracklet*> trackletsVec;
	
	for(int i = 0; i < nTrack; i++)
	{
		Tracklets::tTracklet *tracklet;
		tracklet = tracklets->getTracklet(i);
		trackletsVec.push_back(tracklet);
	}
	std::cerr << trackletsVec[1]->h << std::endl;
	std::cerr << trackletsVec[1]->poses[1].tx << std::endl;

	delete tracklets;

	float h = 1.4778554;
	float w = 1.8019032;
	float l = 4.5137277;

	double tx = 7.2628571625253793;
	double ty = 3.8923224413722752;
	double tz = -1.7135605745017528;

	double rx = 0;
	double ry = 0;
	double rz = -3.1269888029050974;

	double minX = 0 - (l / 2);
	double minY = 0 - (w / 2);
	double minZ = 0;

	double maxX = (l / 2);
	double maxY = (w / 2);
	double maxZ = h;

	pcl::CropBox<PointT> filter;
	filter.setInputCloud(cloud_in);
	filter.setMin(Eigen::Vector4f((float)minX, (float)minY, (float)minZ, 1.0));
	filter.setMax(Eigen::Vector4f((float)maxX, (float)maxY, (float)maxZ, 1.0));
	filter.setRotation(Eigen::Vector3f((float)rx, (float)ry, (float)rz));//rotation
	filter.setTranslation(Eigen::Vector3f((float)tx, (float)ty, (float)tz));
	
	filter.filter(*cloud_out);

	PointCloudT cloud_B;
	cloud_B.points = cloud_out.get()->points;
	cloud_B.height = cloud_out.get()->height;
	cloud_B.width = cloud_out.get()->width;

	pcl::io::savePCDFileASCII("0000000000filted10A.pcd", cloud_B);

	return 0;
}
