#pragma once

#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "filter.hpp"

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = std::shared_ptr<CloudT>;

namespace point_cloud_accumulator_pkg
{

  class Accumulator
  {
    public:
      using FilterPtr = std::shared_ptr<Filter>;
      using VoxelScalerPtr = std::shared_ptr<VoxelScaler>;
      Accumulator(double voxel_size_m, FilterPtr in, FilterPtr out, VoxelScalerPtr scaler);
      CloudPtr ingest(const CloudPtr &cloud);
      CloudPtr getAccumulatedCloud() const;
      double getVoxelSize() const;

    private:
      CloudPtr accumulated_cloud_;
      FilterPtr filter_in_;
      FilterPtr filter_out_;
      VoxelScalerPtr scaler_; // FIXME Remove scaler; inject into downsampling filter
      double voxel_size_m_; // FIXME 
  };

} // namespace point_cloud_accumulator_pkg
