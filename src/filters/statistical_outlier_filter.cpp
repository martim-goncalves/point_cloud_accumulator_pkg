#include "point_cloud_accumulator_pkg/filters/statistical_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"
#include "point_cloud_accumulator_pkg/io/stop_watch.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  StatisticalOutlierFilter::StatisticalOutlierFilter(const std::string &tag, int mean_k, double std_ratio)
    : Filter(tag)
    , mean_k_(mean_k)
    , std_ratio_(std_ratio) 
  {
    // Build header for the logs
    io::Logger::get().logStep(tag,
      "timestamp", "elapsed",       // Time
      "pts_kept", "pts_filtered",   // Information
      "mean_k", "std_ratio"         // Params
    );
  }
  
  CloudPtr StatisticalOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {
    // Set initial timestamp
    auto& stopwatch = io::StopWatch::get();
    auto [start, t0] = stopwatch.now();

    // [Guard Clause] :: Skip processing empty clouds
    if (cloud->empty())
      return cloud;

    // Apply SOR
    auto filtered = std::make_shared<CloudT>();
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(std_ratio_);
    sor.filter(*filtered);

    // Build record for the current step
    auto [end, t] = stopwatch.now();
    io::Logger::get().logStep(tag_,
      stopwatch.getTimestamp(t0),
      stopwatch.getElapsedMicros(start, end),
      filtered->size(),
      cloud->size() - filtered->size(),
      mean_k_, 
      std_ratio_
    );

    return filtered;
  }

} // namespace point_cloud_accumulator_pkg::filters
