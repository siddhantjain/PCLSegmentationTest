#ifndef ISegAlgorithms_h
#define ISegAlgorithms_h

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class ISegAlgorithm
{
public:
	virtual pcl::PointCloud <pcl::PointXYZRGB>::Ptr Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud) = 0; 
protected:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pSegmentedCloud;
};
#endif