#pragma once

#include <deque>
#include <algorithm>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  class ColorConsistencyFilter : public Filter
  {
    public:
      ColorConsistencyFilter(const std::string &tag, int history_size = 10, uint8_t saturation_threshold = 240);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;

    private:
      bool isOverexposed(const PointT &pt) const;
      void normalizeBrightness(PointT &pt, float target_brightness) const;
      float averageBrightness(const CloudPtr& cloud) const;
      float rollingAverage() const;

      mutable std::deque<float> brightness_history_;
      int history_size_;
      uint8_t saturation_threshold_;
  };

} // namespace point_cloud_accumulator_pkg::filters
