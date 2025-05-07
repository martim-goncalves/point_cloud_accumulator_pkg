#pragma once

#include "point_cloud_accumulator_pkg/curve.hpp"

namespace point_cloud_accumulator_pkg::curves
{

  class LogisticSigmoid : public Curve
  {

    public:
      LogisticSigmoid(size_t thr1, size_t thr2, double ymin, double ymax, double steepness = 5.0);
      double f(size_t x) const override;

    private:
      size_t thr1_, thr2_;
      double ymin_, ymax_, steepness_;

  };

} // namespace point_cloud_accumulator_pkg::curves
