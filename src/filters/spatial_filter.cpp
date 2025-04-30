#include "point_cloud_accumulator_pkg/filters/spatial_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  SpatialFilter::SpatialFilter(float distance_thr_m, int min_neighbors)
    : distance_thr_m_(distance_thr_m)
    , min_neighbors_(min_neighbors)
  {}

  CloudPtr SpatialFilter::applyFilter(const CloudPtr &cloud) const
  {

    CloudPtr filtered = std::make_shared<CloudT>();
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> sq_distances;

    for (const auto &point : cloud->points)
    {
      if (kdtree.radiusSearch(point, distance_thr_m_, indices, sq_distances) >= min_neighbors_)
        filtered->points.push_back(point);
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;

    return filtered;

  }

} // namespace point_cloud_accumulator_pkg::filters
