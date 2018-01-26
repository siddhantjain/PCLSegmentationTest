#ifndef SegAlgorithms_h
#define SegAlgorithms_h

#include "ISegAlgorithm.h"

class SegmentationAlgorithm
{
public:
	SegmentationAlgorithm(int algorithmCode);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud);
};

#endif
