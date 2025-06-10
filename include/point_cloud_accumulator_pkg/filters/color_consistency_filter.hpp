#pragma once

#include <deque>
#include <algorithm>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  /**
   * Plan for color normalization:
   * - VoxelGrid for downsampling
   * - ColorNormalizationFilter
   * 
   * Using a VoxelGrid for normalization has the benefit of placing each point at known locations. By applying it to 
   * each point cloud frame and then comparing it to the mean and standard deviation values we may be able to pull 
   * normalization off.
   * 
   * The VoxelGrid's resolution should be greater than any of the final maps, be it the fused cloud or the octomap. 
   * The ColorNormalizationFilter should keep a data structure similar to the accumulated cloud where for each XYZ 
   * there are mean RGB values and their standard deviation for each point.
   */
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
