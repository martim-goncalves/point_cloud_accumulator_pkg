#include "point_cloud_accumulator_pkg/filters/nan_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"
#include "point_cloud_accumulator_pkg/io/stop_watch.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  NaNFilter::NaNFilter(const std::string &tag) : Filter(tag)
  {
    // Build header for the logs
    io::Logger::get().logStep(tag_, "timestamp", "elapsed", "pts_kept", "pts_filtered");
  }

  CloudPtr NaNFilter::applyFilter(const CloudPtr &cloud) const
  {
    // Set initial timestamp
    auto& stopwatch = io::StopWatch::get();
    auto [start, t0] = stopwatch.now();

    // [Guard Clause] :: Skip processing empty clouds
    if (cloud->empty()) return cloud;

    // Remove NaN
    auto filtered = std::make_shared<CloudT>();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filtered, indices);
    
    // Calculate statistics
    auto total_pts = cloud->size();
    auto pts_kept = filtered->size();
    auto pts_filtered = total_pts - pts_kept;

    // Build record for the current step
    auto [end, t] = stopwatch.now();
    io::Logger::get().logStep(tag_,
      stopwatch.getTimestamp(t0), 
      stopwatch.getElapsedMicros(start, end), 
      pts_kept, 
      pts_filtered
    );
    
    return filtered;
  }

} // namespace point_cloud_accumulator_pkg::filters
