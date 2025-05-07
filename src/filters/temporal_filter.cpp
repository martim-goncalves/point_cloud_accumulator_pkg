#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud_accumulator_pkg/filters/temporal_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  TemporalFilter::TemporalFilter(
    const std::string &tag, 
    int history_size, 
    float distance_thr_m, 
    float min_appearance_ratio
  ) : Filter(tag)
    , history_size_(history_size)
    , distance_thr_m_(distance_thr_m)
    , min_appearance_ratio_(min_appearance_ratio)
  {
    // Build header for the logs
    auto& logger = io::Logger::get();
    logger.logStep(tag, logger.makeRecord(
      "timestamp", "elapsed",                                   // Time
      "pts_kept", "pts_filtered",                               // Information
      "distance_thr_m", "min_appearance_ratio", "history_size"  // Params
    ));
  }

  CloudPtr TemporalFilter::applyFilter(const CloudPtr &cloud) const
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

    // Add current cloud to history
    if (frame_history_.size() >= history_size_)
      frame_history_.pop_front();
    frame_history_.push_back(cloud);

    // [Guard Clause] :: Skip filtering if the history is too short
    if (frame_history_.size() < 2)
      return cloud;

    // Collect stable points
    CloudPtr stable_cloud = std::make_shared<CloudT>();
    for (const auto &point : cloud->points)
    {
      if (isPointStable(point))
        stable_cloud->points.push_back(point);
    }

    // Update stable point cloud properties
    stable_cloud->width = stable_cloud->points.size();
    stable_cloud->height = 1;
    stable_cloud->is_dense = false;

    // Get elapsed time in milliseconds
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Build record for the current step
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord(
      timestamp,                              //
      millis,                                 //
      stable_cloud->size(),                   // Points kept
      cloud->size() - stable_cloud->size(),   // Points filtered
      distance_thr_m_,                        // 
      min_appearance_ratio_,                  // 
      history_size_                           // 
    ));

    return stable_cloud;

  }

  void TemporalFilter::setDistanceThreshold(float distance_thr) 
  {
    distance_thr_m_ = distance_thr;
  }

  bool TemporalFilter::isPointStable(const PointT& pt) const
  {

    int appearances = 0;

    for (const auto &past_cloud : frame_history_)
    {
      pcl::KdTreeFLANN<PointT> kdtree;
      kdtree.setInputCloud(past_cloud);

      std::vector<int> indices;
      std::vector<float> sq_dists;

      if (kdtree.radiusSearch(pt, distance_thr_m_, indices, sq_dists) > 0)
        ++appearances;
    }

    float ratio = static_cast<float>(appearances) / frame_history_.size();
    return ratio >= min_appearance_ratio_;

  }

} // namespace point_cloud_accumulator_pkg::filters
