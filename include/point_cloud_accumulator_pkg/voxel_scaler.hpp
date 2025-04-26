#pragma once

#include <cstddef>
#include <memory>
#include "curve.hpp"

namespace point_cloud_accumulator_pkg 
{

  class VoxelScaler 
  {

    public: 
      using CurvePtr = std::unique_ptr<Curve>;
      VoxelScaler(size_t start, size_t end, double min_size, double max_size, CurvePtr fn);
      double getVoxelSize(size_t num_points) const;

    private:
      size_t min_points_thr_;
      size_t max_points_thr_;
      double min_voxel_size_;
      double max_voxel_size_;
      CurvePtr curve_;

  };

} // namespace point_cloud_accumulator_pkg 
