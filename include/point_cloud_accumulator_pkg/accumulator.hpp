#pragma once

#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = std::shared_ptr<CloudT>;

// [NOTE] :: Forward declarations speed compilation up. Use them when headers don't need to be included. 
class Filter;
class VoxelScaler;

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
