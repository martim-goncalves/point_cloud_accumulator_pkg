#include <stdexcept>
#include <pcl/filters/voxel_grid.h>

#include "point_cloud_accumulator_pkg/accumulator.hpp"

namespace point_cloud_accumulator_pkg
{

  Accumulator::Accumulator(double voxel_size_m, FilterPtr pipeline, VoxelScalerPtr scaler)
    : voxel_size_m_(voxel_size_m)
    , pipeline_(std::move(pipeline))
    , scaler_(std::move(scaler))
  {

    accumulated_cloud_ = std::make_shared<CloudT>();

  }

  CloudPtr Accumulator::ingest(const CloudPtr &cloud)
  {

    // Protect against silliness.
    if (!cloud)
      throw std::invalid_argument("Input cloud must not be null.");

    // Apply input filter pipeline to ingested frame if available.
    auto filtered_frame = (pipeline_) ? pipeline_->apply(cloud) : cloud;

    // Accumulate and downsample the cloud.
    *accumulated_cloud_ += *filtered_frame;
    downsample(accumulated_cloud_);
    voxel_size_m_ = scaler_->getVoxelSize(accumulated_cloud_->size());

    // Return intermediate filtered frame for publishing, visualization, etc.
    return filtered_frame;

  }

  CloudPtr Accumulator::getAccumulatedCloud() const
  {
    return accumulated_cloud_;
  }

  double Accumulator::getVoxelSize() const
  {
    return voxel_size_m_;
  }

  void Accumulator::downsample(CloudPtr &cloud)
  {
    auto downsampled_cloud = std::make_shared<CloudT>();
    pcl::VoxelGrid<pcl::PointXYZRGB> voxgrid;
    voxgrid.setInputCloud(cloud);
    voxgrid.setLeafSize(voxel_size_m_, voxel_size_m_, voxel_size_m_);
    voxgrid.filter(*downsampled_cloud);
    cloud = downsampled_cloud;
  }

} // namespace point_cloud_accumulator_pkg
