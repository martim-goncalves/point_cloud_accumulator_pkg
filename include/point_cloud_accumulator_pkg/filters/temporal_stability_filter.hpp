#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>

#include "point_cloud_accumulator_pkg/filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  struct VoxelData
  {
    int hits = 0;
    Eigen::Vector4f sum_pos = Eigen::Vector4f::Zero();
    float sum_r = 0.0f;
    float sum_g = 0.0f;
    float sum_b = 0.0f;
  };

  class TemporalStabilityFilter : public Filter
  {
    public:
      TemporalStabilityFilter(float leaf_size, int hit_thr)
        : Filter("TemporalVoxel")
        , leaf_size_(leaf_size)
        , hit_thr_(hit_thr)
      {}

    protected:
      CloudPtr applyFilter(const CloudPtr &cloud) const override
      {
        for (const auto &pt : *cloud)
        {
          int ix = static_cast<int>(std::floor(pt.x / leaf_size_));
          int iy = static_cast<int>(std::floor(pt.y / leaf_size_));
          int iz = static_cast<int>(std::floor(pt.z / leaf_size_));

          VoxelKey key = std::make_tuple(ix, iy, iz);
          auto &data = voxel_map_[key];
          data.hits++;
          data.sum_pos += pt.getVector4fMap();

          uint8_t r = pt.r;
          uint8_t g = pt.g;
          uint8_t b = pt.b;

          data.sum_r += static_cast<float>(r);
          data.sum_g += static_cast<float>(g);
          data.sum_b += static_cast<float>(b);
        }

        CloudPtr filtered(new CloudT);
        for (const auto &[key, data] : voxel_map_)
        {
          if (data.hits >= hit_thr_)
          {
            Eigen::Vector4f avg = data.sum_pos / static_cast<float>(data.hits);
            float r = data.sum_r / static_cast<float>(data.hits);
            float g = data.sum_g / static_cast<float>(data.hits);
            float b = data.sum_b / static_cast<float>(data.hits);

            PointT pt;
            pt.x = avg.x();
            pt.y = avg.y();
            pt.z = avg.z();
            filtered->push_back(pt);
          }
        }

        return filtered;
      }

    private:
      using VoxelKey = std::tuple<int, int, int>;
      struct VoxelHash
      {
        std::size_t operator()(const VoxelKey &k) const
        {
          auto [x, y, z] = k;
          return std::hash<int>()(x) ^ std::hash<int>()(y << 1) ^ std::hash<int>()(z << 2);
        }
      };
      
      float leaf_size_;
      int hit_thr_;
      mutable std::unordered_map<VoxelKey, VoxelData, VoxelHash> voxel_map_;
  };
} // namespace point_cloud_accumulator_pkg::filters
