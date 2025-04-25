#pragma once

#include <memory>
#include <stdexcept>
#include "filter.hpp"
#include "voxel_scaler.hpp"

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

      Accumulator(double voxel_size_m, VoxelScalerPtr scaler, FilterPtr in, FilterPtr out)
        : voxel_size_m_(voxel_size_m)
        , filter_in_(std::move(in))
        , filter_out_(std::move(out))
        , scaler_(std::move(scaler))
      {

        if (!(in && out))
          throw std::invalid_argument("The filter pipelines must not be null.");

        accumulated_cloud_ = std::make_shared<CloudT>();

      }

      CloudPtr ingest(CloudPtr cloud)
      {

        // Apply input filter to ingested frame.
        auto filtered_frame = filter_in_->apply(cloud);
        
        // Accumulate and downsample the cloud.
        // TODO Complete and consider how to integrate octree data structure with spatial and temporal filters.
        voxel_size_m_ = scaler_->getVoxelSize(accumulated_cloud_->size());

        // Return intermediate filtered frame for publishing, visualization, etc.
        return filtered_frame;
      }

      CloudPtr getAccumulatedCloud() const 
      {
        return accumulated_cloud_;
      }

      double getVoxelSize() const 
      {
        return voxel_size_m_;
      }

    private:

      CloudPtr accumulated_cloud_;
      FilterPtr filter_in_;
      FilterPtr filter_out_;
      VoxelScalerPtr scaler_;
      double voxel_size_m_;

  };

}