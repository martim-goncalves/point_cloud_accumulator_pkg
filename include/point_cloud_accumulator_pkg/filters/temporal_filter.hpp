#pragma once

#include <deque>
#include <memory>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  /**
   * This filter, in contrast to the spatial filter, guarantees point cloud integrity and consistency accross frames 
   * (temporal axis).
   */
  class TemporalFilter : public Filter
  {
    public:
      TemporalFilter(int history_size = 5, float distance_thr_m = 0.02f, float min_appearance_ratio = 0.6f);
      void setDistanceThreshold(float distance_thr);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;

    private:
      bool isPointStable(const PointT &pt) const;

      /** Double ended queue keeping a limited history of point cloud frames. */
      mutable std::deque<CloudPtr> frame_history_;
      /** Size limit of the point cloud frame history. */
      int history_size_;
      /** Distance threshold, in meters, for a point to be considered the same across frames. */
      float distance_thr_m_;
      /** Frequency (as a ratio) of occurences for a point to be persisted. */
      float min_appearance_ratio_;
  };

} // namespace point_cloud_accumulator_pkg::filters
