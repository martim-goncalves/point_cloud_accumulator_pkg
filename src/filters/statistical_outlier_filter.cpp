#include "point_cloud_accumulator_pkg/filters/statistical_outlier_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  StatisticalOutlierFilter::StatisticalOutlierFilter(int mean_k, double std_ratio)
    : mean_k_(mean_k)
    , std_ratio_(std_ratio) 
  {}
  
  CloudPtr StatisticalOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {
    auto filtered = std::make_shared<CloudT>();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_ratio_);
    sor.filter(*filtered);
    return filtered;
  }

} // namespace point_cloud_accumulator_pkg::filters
