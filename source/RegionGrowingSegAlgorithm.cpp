#include "includes/RegionGrowingSegAlgorithm.h"



RegionGrowingSegAlgorithm::RegionGrowingSegAlgorithm()
{
    //TODO: move all constants as an argument with default values as these
    m_rgSegmenter =  new pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>();
    m_rgSegmenter->setMinClusterSize (50);
    m_rgSegmenter->setMaxClusterSize (1000000);
    m_rgSegmenter->setNumberOfNeighbours (30);
    m_rgSegmenter->setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    m_rgSegmenter->setCurvatureThreshold (1.0);
}

pcl::PointCloud <pcl::PointXYZRGB>::Ptr RegionGrowingSegAlgorithm::Segment(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud)
{
    m_pCloud = inCloud;
    pcl::PointCloud <pcl::Normal>::Ptr normals = CalculateNormalsAndFilterPoints();
    m_rgSegmenter->setInputCloud(m_pCloud);
    m_rgSegmenter->setInputNormals (normals);
    std::vector <pcl::PointIndices> clusters;
    m_rgSegmenter->extract (clusters);
    
    return m_rgSegmenter->getColoredCloud ();
}

pcl::PointCloud <pcl::Normal>::Ptr RegionGrowingSegAlgorithm::CalculateNormalsAndFilterPoints()
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (m_pCloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
    
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (m_pCloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);
    
    return normals;
}

