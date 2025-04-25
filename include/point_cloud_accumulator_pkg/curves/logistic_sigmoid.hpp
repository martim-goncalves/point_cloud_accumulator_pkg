#pragma once

#include <stdexcept>
#include <complex>
#include "curve.hpp"

namespace point_cloud_accumulator_pkg::curves
{

  class LogisticSigmoid : public Curve
  {

    public:

      LogisticSigmoid(size_t thr1, size_t thr2, double ymin, double ymax, double steepness) 
      : thr1_(thr1)
      , thr2_(thr2)
      , ymin_(ymin)
      , ymax_(ymax)
      , steepness_(steepness)
      {

        if (thr2 <= thr1)
          throw std::invalid_argument("thr2 must be > thr1");
        
        if (steepness <= 0.0) 
          throw std::invalid_argument("Steepness factor must be greater than zero.");

      }

      double f(size_t x) const override 
      {
        auto midpoint = (thr1_+ thr2_) / 2.0;
        auto k = steepness_ / (thr2_ - thr1_);
        double s = 1.0 / (1.0 + std::exp(-k * (x - midpoint)));
        return ymin_ + s * (ymax_ - ymin_);
      }

    private:

      size_t thr1_, thr2_;
      double ymin_, ymax_, steepness_;

  };

}