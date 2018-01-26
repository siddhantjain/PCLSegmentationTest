#include <iostream>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

//remove this when we have a better class defintion

#include "includes/SegmentationAlgorithm.h"
#include "includes/RegionGrowingSegAlgorithm.h"

#define _VIEW_ON

int main ()
{
	
	const std::string FILE_NAME = "../data/region_growing_tutorial.pcd";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	if ( pcl::io::loadPCDFile <pcl::PointXYZ> (FILE_NAME, *cloud) == -1)
	{
    	std::cout << "Cloud reading failed." << std::endl;
    	return (-1);
  }


  	
  	//Logic for doing region growing algorithm 
  RegionGrowingSegAlgorithm* pSegAlgorithm = new RegionGrowingSegAlgorithm();
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
  colored_cloud = pSegAlgorithm->Segment(cloud);
  	
  #ifdef _VIEW_ON
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
  

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
  #endif




}
