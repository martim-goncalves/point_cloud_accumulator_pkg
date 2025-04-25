#pragma once
#include <cstddef>
#include <stdexcept>
#include <memory>
#include "curve.hpp"

namespace point_cloud_accumulator_pkg 
{

  class VoxelScaler 
  {

    public: 

      VoxelScaler(size_t start, size_t end, double min_size, double max_size, std::unique_ptr<Curve> fn) 
        : min_points_thr_(start)
        , max_points_thr_(end) 
        , min_voxel_size_(min_size)
        , max_voxel_size_(max_size)
        , curve_(std::move(fn))
      {

        if (start <= 0 || end <= 0 || min_size <= 0 || max_size <= 0)
          throw std::invalid_argument("All thresholds and sizes must be positive.");

        if (start >= end)
          throw std::invalid_argument("Start threshold must be < end threshold.");

        if (min_size > max_size)
          throw std::invalid_argument("Minimum voxel size cannot be greater than maximum voxel size.");

        if (!fn)
          throw std::invalid_argument("The interpolation curve function must not be null.");

      }

      [[nodiscard]]
      double getVoxelSize(size_t num_points) 
      {
        auto clamped_points = std::min(std::max(num_points, min_points_thr_), max_points_thr_);
        return curve_->f(clamped_points);
      }
    
    private:

      size_t min_points_thr_;
      size_t max_points_thr_;
      double min_voxel_size_;
      double max_voxel_size_;
      std::unique_ptr<Curve> curve_;

  };

}
