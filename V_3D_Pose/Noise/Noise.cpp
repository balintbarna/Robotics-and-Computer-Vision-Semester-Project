#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <ctime>
#include <cstdlib>  


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::visualization;

int main (int argc, char** argv)
{
	
    if(argc < 2) {
		cout << "Usage: " << argv[0] << "<scene> [noise] " << endl;
        return 0;
    }
	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


	pcl::PCDReader readerpcd;
	readerpcd.read (argv[1], *cloud);
	
	{
        PCLVisualizer v("Input");
        v.addPointCloud<PointXYZ>(cloud, PointCloudColorHandlerCustom<PointXYZ>(cloud, 0, 255, 0), "scene");
        v.spin();
    }
	
	for(size_t i=0; i < cloud->points.size (); ++i)
	{
	cloud->points[i].x += ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))/std::stoi(argv[2])); 
	cloud->points[i].y += ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))/std::stoi(argv[2]));
	cloud->points[i].z += ((static_cast <float> (rand()) / static_cast <float> (RAND_MAX))/std::stoi(argv[2]));
	} 
	
	{
        PCLVisualizer v("Noise");
        v.addPointCloud<PointXYZ>(cloud, PointCloudColorHandlerCustom<PointXYZ>(cloud, 0, 255, 0), "scene");
        v.spin();
    }

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("noise.pcd", *cloud, false);
	
	return (0);
}

