#pragma once

#include <pcl/filters/statistical_outlier_removal.h>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  class StatisticalOutlierFilter : public Filter
  {
  public:
    StatisticalOutlierFilter(const std::string &tag, int mean_k = 20, double std_ratio = 2.0);

  protected:
    CloudPtr applyFilter(const CloudPtr &cloud) const override;

  private:
    /** Mean K neighbors a point must have. */
    int mean_k_;
    /** Standard deviation multiplier for distance threshold calculation. */
    double std_ratio_;
  };

} // namespace point_cloud_accumulator_pkg::filters
