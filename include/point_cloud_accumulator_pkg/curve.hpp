#pragma once
#include <cstddef>

namespace point_cloud_accumulator_pkg
{

  /**
   * @brief Abstract interface representing an interpolation curve.
   *
   * Implementations map an arbitrary integer number (x) to a scalar (y). 
   */
  class Curve
  {
    public:
      virtual ~Curve() = default;
      virtual double f(size_t x) const = 0;
  };

}