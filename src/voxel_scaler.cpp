#include <stdexcept>
#include <algorithm>
#include "point_cloud_accumulator_pkg/voxel_scaler.hpp"

namespace point_cloud_accumulator_pkg
{

  VoxelScaler::VoxelScaler(size_t start, size_t end, double min_size, double max_size, CurvePtr fn)
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

  double VoxelScaler::getVoxelSize(size_t num_points) const
  {
    // auto clamped_points = std::min(std::max(num_points, min_points_thr_), max_points_thr_);
    auto clamped_points = std::clamp(num_points, min_points_thr_, max_points_thr_);
    return curve_->f(clamped_points);
  }

} // namespace point_cloud_accumulator_pkg 
