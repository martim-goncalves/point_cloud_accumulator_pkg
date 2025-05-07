#include <ctime>
#include <chrono>
#include <numeric>

#include "point_cloud_accumulator_pkg/filters/spatial_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  SpatialFilter::SpatialFilter(const std::string &tag, float distance_thr_m, int min_neighbors)
    : Filter(tag)
    , distance_thr_m_(distance_thr_m)
    , min_neighbors_(min_neighbors)
  {
    // Build header for the logs
    auto& logger = io::Logger::get();
    logger.logStep(tag, logger.makeRecord(
      "timestamp", "elapsed",                           // Time
      "pts_kept", "pts_filtered", "mean_dist", "std",   // Information
      "dist_thr_m", "min_neighbors"                     // Params
    ));
  }

  CloudPtr SpatialFilter::applyFilter(const CloudPtr &cloud) const
  {

    // Set initial timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
    std::string timestamp = time_buf;

    CloudPtr filtered = std::make_shared<CloudT>();
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices;
    std::vector<float> sq_distances;

    for (const auto &point : cloud->points)
    {
      if (kdtree.radiusSearch(point, distance_thr_m_, indices, sq_distances) >= min_neighbors_)
        filtered->points.push_back(point);
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;

    // Get elapsed time in milliseconds
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // ...
    auto distances = nearestNeighborDistances(cloud);
    auto [mean_dist, std_dist] = distanceStats(distances);

    // Build record for the current step
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord(
      timestamp,                          //
      millis,                             //
      filtered->size(),                   // Points kept
      cloud->size() - filtered->size(),   // Points filtered
      mean_dist,                          // Mean distance between points
      std_dist,                           // Mean distance standard deviation
      distance_thr_m_,                    // 
      min_neighbors_                      // 
    ));

    return filtered;

  }

  std::vector<float> SpatialFilter::nearestNeighborDistances(const CloudPtr &cloud, int k) const
  {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<float> distances;
    std::vector<int> pointIdxNKNSearch(k);
    std::vector<float> pointNKNSquaredDistance(k);
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (kdtree.nearestKSearch(cloud->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        distances.push_back(std::sqrt(pointNKNSquaredDistance[1])); // [0] is the point itself
      }
    }
    return distances;
  }
  
  std::pair<double, double> SpatialFilter::distanceStats(const std::vector<float> &distances) const
  {
    double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0);
    double std_dev = std::sqrt(sq_sum / distances.size() - mean * mean);
    return {mean, std_dev};
  }

} // namespace point_cloud_accumulator_pkg::filters
