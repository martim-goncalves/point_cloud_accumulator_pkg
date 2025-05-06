#include "point_cloud_accumulator_pkg/filters/color_consistency_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  // FIXME Brightness should be normalized on the temporal axis for each point, not globally or shadows and bright spots will get eaten up

  ColorConsistencyFilter::ColorConsistencyFilter(
    const std::string &tag, 
    int history_size, 
    uint8_t saturation_threshold
  ) : Filter(tag)
    , history_size_(history_size)
    , saturation_threshold_(saturation_threshold)
  {
    // TODO Log headers: timestamp, elapsed, ..., number of overexposed points, saturation_thr, history_size
  }

  CloudPtr ColorConsistencyFilter::applyFilter(const CloudPtr& cloud) const
  {

    CloudPtr filtered = std::make_shared<CloudT>();
    filtered->reserve(cloud->size());

    float avg_brightness = averageBrightness(cloud);
    brightness_history_.push_back(avg_brightness);
    if (brightness_history_.size() > history_size_)
      brightness_history_.pop_front();

    float target_brightness = rollingAverage();

    for (const auto &point : cloud->points)
    {
      if (isOverexposed(point)) 
        continue;
      PointT corrected_point = point;
      normalizeBrightness(corrected_point, target_brightness);
      filtered->points.push_back(corrected_point);
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;

  }

  bool ColorConsistencyFilter::isOverexposed(const PointT &point) const
  {
    return point.r > saturation_threshold_ && point.g > saturation_threshold_ && point.b > saturation_threshold_;
  }

  void ColorConsistencyFilter::normalizeBrightness(PointT &point, float target) const
  {

    float brightness = (point.r + point.g + point.b) / 3.0f;
    if (brightness < 1.0f) 
      brightness = 1.0f;

    float scale = target / brightness;
    point.r = static_cast<uint8_t>(std::clamp(point.r * scale, 0.0f, 255.0f));
    point.g = static_cast<uint8_t>(std::clamp(point.g * scale, 0.0f, 255.0f));
    point.b = static_cast<uint8_t>(std::clamp(point.b * scale, 0.0f, 255.0f));

  }

  float ColorConsistencyFilter::averageBrightness(const CloudPtr &cloud) const
  {

    float sum = 0.0f;
    int count = 0;

    for (const auto &point : cloud->points)
    {
      if (!isOverexposed(point))
      {
        sum += (point.r + point.g + point.b) / 3.0f;
        ++count;
      }
    }

    return count > 0 ? sum / count : 128.0f; // fallback to neutral brightness

  }

  float ColorConsistencyFilter::rollingAverage() const
  {
    float sum = 0.0f;
    for (float b : brightness_history_)
      sum += b;
    return brightness_history_.empty() ? 128.0f : sum / brightness_history_.size();
  }

} // namespace point_cloud_accumulator_pkg::filters
