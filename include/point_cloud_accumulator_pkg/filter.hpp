#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudPtr = std::shared_ptr<CloudT>;

namespace point_cloud_accumulator_pkg
{

  /**
   * @brief Abstract class for processing pipelines.
   */
  class Filter
  {

    public:

      /** Alias for shared pointers to Filter instances. */
      using FilterPtr = std::shared_ptr<Filter>;

      virtual ~Filter() = default;

      /** Sets the successor (next filter) in the processing pipeline.  */
      void setNext(FilterPtr filter) { next_ = filter; }

      /**
       * @brief Applies the filter to a point cloud.
       * @param cloud A point cloud frame.
       */
      CloudPtr apply(const CloudPtr& cloud) 
      {
        CloudPtr filtered_cloud = applyFilter(cloud);
        if (next_)
        {
          return next_->apply(filtered_cloud);
        }
        return filtered_cloud;
      }

    protected:

      /**
       * @brief Abstract method which opens filters up for specialization.apply
       * 
       * Filter specializations should implement this method to customize processing behavior.
       */
      virtual CloudPtr applyFilter(const CloudPtr& cloud) const = 0;

    private:

      /** Successor filter in the processing pipeline. */
      FilterPtr next_;

  };

} // namespace point_cloud_accumulator_pkg
