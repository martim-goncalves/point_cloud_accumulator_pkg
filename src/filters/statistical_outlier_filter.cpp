#include "point_cloud_accumulator_pkg/filters/statistical_outlier_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  StatisticalOutlierFilter::StatisticalOutlierFilter(const std::string &tag, int mean_k, double std_ratio)
    : Filter(tag)
    , mean_k_(mean_k)
    , std_ratio_(std_ratio) 
  {
    // TODO Log headers: timestamp, elapsed, total points, points kept, points filtered, mean neighbors, standard deviation from mean neighbors, mean_k, std_ratio
  }
  
  CloudPtr StatisticalOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {

    auto filtered = std::make_shared<CloudT>();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_ratio_);
    sor.filter(*filtered);

    // TODO Log timestamp, elapsed, total points, points kept, points filtered, mean neighbors, standard deviation from mean neighbors, mean_k, std_ratio

    return filtered;

  }

} // namespace point_cloud_accumulator_pkg::filters
