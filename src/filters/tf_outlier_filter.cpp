#include "point_cloud_accumulator_pkg/filters/tf_outlier_filter.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  TFOutlierFilter::TFOutlierFilter(double max_translation_m, double max_rotation_deg, int history_size)
    : max_translation_m_(max_translation_m)
    , max_rotation_deg_(max_rotation_deg)
    , history_size_(history_size)
  {}

  void TFOutlierFilter::setCurrentTransform(const Eigen::Affine3f &tf)
  {
    current_tf_ = tf;
    has_tf_ = true;
  }

  CloudPtr TFOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {

    if (!has_tf_)
      return cloud;

    if (!isValid(current_tf_))
      return nullptr;

    if (tf_history_.size() >= history_size_)
      tf_history_.pop_front();

    tf_history_.push_back(current_tf_);

    return cloud;
  
  }

  /**
   * @brief Validates if a transform is within the constraints set for the filter.
   * @param tf The transform being validated.
   * @return True if the transform is valid.
   */
  bool TFOutlierFilter::isValid(const Eigen::Affine3f &tf) const
  {

    if (tf_history_.empty())
      return true;

    const Eigen::Affine3f &last_tf = tf_history_.back();
    Eigen::Vector3f translation_diff = tf.translation() - last_tf.translation();
    double translation_mag = translation_diff.norm();

    double angle_deg = getRotationAngleDeg(tf.rotation(), last_tf.rotation());

    return translation_mag <= max_translation_m_ && angle_deg <= max_rotation_deg_;
  
  }

  double TFOutlierFilter::getRotationAngleDeg(const Eigen::Matrix3f &a, const Eigen::Matrix3f &b) const
  {
    Eigen::Matrix3f delta = a.transpose() * b;
    double trace = std::max(-1.0f, std::min(1.0f, (delta.trace() - 1.0f) / 2.0f));
    double angle_rad = std::acos(trace);
    return angle_rad * 180.0 / M_PI;
  }

} // namespace point_cloud_accumulator_pkg::filters
