#include "point_cloud_accumulator_pkg/filters/nan_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  NaNFilter::NaNFilter(const std::string &tag) : Filter(tag)
  {
    // Build header for the logs
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord("timestamp", "elapsed", "pts_kept", "pts_filtered"));
  }

  CloudPtr NaNFilter::applyFilter(const CloudPtr &cloud) const
  {
    // Set initial timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
    std::string timestamp = time_buf;

    // [Guard Clause] :: Skip processing empty clouds
    if (cloud->empty()) return cloud;

    // Remove NaN
    auto filtered = std::make_shared<CloudT>();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);

    // Get elapsed time in milliseconds
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    
    // Calculate statistics
    auto total_pts = cloud->size();
    auto pts_kept = filtered->size();
    auto pts_filtered = total_pts - pts_kept;

    // Build record for the current step
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord(timestamp, millis, pts_kept, pts_filtered));
    
    return filtered;
  }

} // namespace point_cloud_accumulator_pkg::filters
