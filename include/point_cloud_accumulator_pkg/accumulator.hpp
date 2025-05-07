#pragma once

#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "point_cloud_accumulator_pkg/filter.hpp"
#include "point_cloud_accumulator_pkg/voxel_scaler.hpp"

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
      Accumulator(double voxel_size_m, FilterPtr pipeline, VoxelScalerPtr scaler);
      CloudPtr ingest(const CloudPtr &cloud);
      CloudPtr getAccumulatedCloud() const;
      double getVoxelSize() const;

    private:
      void downsample(CloudPtr &cloud);

      CloudPtr accumulated_cloud_;
      FilterPtr pipeline_;
      VoxelScalerPtr scaler_;
      double voxel_size_m_;
  };

} // namespace point_cloud_accumulator_pkg
