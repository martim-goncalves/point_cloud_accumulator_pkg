#pragma once

#include <pcl/filters/filter.h>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{
  class NaNFilter : public Filter
  {
    public:
      NaNFilter(const std::string &tag);

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override;
  };
} // namespace point_cloud_accumulator_pkg::filters
