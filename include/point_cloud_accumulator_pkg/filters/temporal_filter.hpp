#pragma once

#include <deque>
#include <memory>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  class TemporalFilter : public Filter
  {

    public:
      TemporalFilter(size_t history_size = 5, float distance_thr_m = 0.02f, float min_appearance_ratio = 0.6f);
      void setDistanceThreshold(float distance_thr);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;

    private:
      bool isPointStable(const PointT &pt) const;

      mutable std::deque<CloudPtr> frame_history_;
      size_t history_size_;
      float distance_thr_m_;
      float min_appearance_ratio_;

  };

} // namespace point_cloud_accumulator_pkg::filters
