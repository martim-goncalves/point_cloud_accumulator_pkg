#pragma once

#include <deque>
#include <Eigen/Dense>
#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  class TfOutlierFilter : public Filter
  {

  public:
    TfOutlierFilter(double max_trans_m = 0.3, double max_rot_deg = 15.0, uint8_t history_size = 5);
    void setCurrentTransform(const Eigen::Affine3f& tf);

  protected:
    CloudPtr applyFilter(const CloudPtr& cloud) const = 0;

  private:
    bool isValid(const Eigen::Affine3f& tf) const;
    double getRotAngleDeg(const Eigen::Matrix3f& a, const Eigen::Matrix3f& b) const;

  };

} // namespace point_cloud_accumulator_pkg::filters
