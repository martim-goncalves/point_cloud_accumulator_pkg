#include <stdexcept>

#include "point_cloud_accumulator_pkg/filter.hpp"
#include "point_cloud_accumulator_pkg/accumulator.hpp"
#include "point_cloud_accumulator_pkg/voxel_scaler.hpp"

namespace point_cloud_accumulator_pkg
{

  Accumulator::Accumulator(double voxel_size_m, FilterPtr in, FilterPtr out, VoxelScalerPtr scaler)
    : voxel_size_m_(voxel_size_m)
    , filter_in_(std::move(in))
    , filter_out_(std::move(out)) // TODO Change of plans: create a private method for downsampling instead
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
    auto filtered_frame = (filter_in_) ? filter_in_->apply(cloud) : cloud;

    // Accumulate and downsample the cloud.
    *accumulated_cloud_ += *filtered_frame;
    accumulated_cloud_ = (filter_out_) ? filter_out_->apply(accumulated_cloud_) : accumulated_cloud_;
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

} // namespace point_cloud_accumulator_pkg
