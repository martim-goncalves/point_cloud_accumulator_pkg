#include "point_cloud_accumulator_pkg/filters/statistical_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  StatisticalOutlierFilter::StatisticalOutlierFilter(const std::string &tag, int mean_k, double std_ratio)
    : Filter(tag)
    , mean_k_(mean_k)
    , std_ratio_(std_ratio) 
  {
    // Build header for the logs
    auto& logger = io::Logger::get();
    logger.logStep(tag, logger.makeRecord(
      "timestamp", "elapsed",       // Time
      "pts_kept", "pts_filtered",   // Information
      "mean_k", "std_ratio"         // Params
    ));
  }
  
  CloudPtr StatisticalOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {

    // Set initial timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
    std::string timestamp = time_buf;

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

    // Get elapsed time in milliseconds
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Build record for the current step
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord(
      timestamp,                          //
      millis,                             //
      filtered->size(),                   // Points kept
      cloud->size() - filtered->size(),   // Points filtered
      mean_k_,                            // 
      std_ratio_                          // 
    ));

    return filtered;

  }

} // namespace point_cloud_accumulator_pkg::filters
