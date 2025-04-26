#pragma once

#include <deque>
#include <Eigen/Dense>
#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  class TFOutlierFilter : public Filter
  {
    public:
      TFOutlierFilter(double max_translation_m = 0.3, double max_rotation_deg = 15.0, uint8_t history_size = 5);
      void setCurrentTransform(const Eigen::Affine3f &tf);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;

    private:
      bool isValid(const Eigen::Affine3f &tf) const;
      double getRotationAngleDeg(const Eigen::Matrix3f &a, const Eigen::Matrix3f &b) const;

      mutable std::deque<Eigen::Affine3f> tf_history_;
      Eigen::Affine3f current_tf_;
      double max_translation_m_;
      double max_rotation_deg_;
      std::size_t history_size_;
      bool has_tf_ = false;
  };

} // namespace point_cloud_accumulator_pkg::filters
