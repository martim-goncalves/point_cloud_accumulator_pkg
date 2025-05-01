#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud_accumulator_pkg/filters/temporal_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  TemporalFilter::TemporalFilter(int history_size, float distance_thr_m, float min_appearance_ratio)
    : history_size_(history_size)
    , distance_thr_m_(distance_thr_m)
    , min_appearance_ratio_(min_appearance_ratio)
  {}

  CloudPtr TemporalFilter::applyFilter(const CloudPtr &cloud) const
  {

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
