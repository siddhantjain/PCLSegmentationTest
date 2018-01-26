
#ifndef RegionGrowingSegAlgorithms_h
#define RegionGrowingSegAlgorithms_h

#include "ISegAlgorithm.h"
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


class RegionGrowingSegAlgorithm : public ISegAlgorithm
{
public:
	RegionGrowingSegAlgorithm();
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
private:
	pcl::PointCloud <pcl::Normal>::Ptr CalculateNormalsAndFilterPoints();
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>* m_rgSegmenter;
};

#endif