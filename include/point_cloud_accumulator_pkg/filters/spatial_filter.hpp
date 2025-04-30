#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  /**
   * This filter, in contrast to the temporal filter, guarantees point cloud integrity and consistency geometrically 
   * (spatial axis).
   */
  class SpatialFilter : public Filter
  {
    public:
      SpatialFilter(float distance_thr_m = 0.05, int min_neighbors = 5);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;

    private:
      float distance_thr_m_;
      int min_neighbors_;
  };

} // namespace point_cloud_accumulator_pkg::filters
