#include "point_cloud_accumulator_pkg/filters/tf_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::filters
{

  TFOutlierFilter::TFOutlierFilter(
    const std::string &tag, 
    double max_translation_m, 
    double max_rotation_deg, 
    int history_size
  ) : Filter(tag)
    , max_translation_m_(max_translation_m)
    , max_rotation_deg_(max_rotation_deg)
    , history_size_(history_size)
  {
    // Build header for the logs
    auto& logger = io::Logger::get();
    logger.logStep(tag, logger.makeRecord(
      "timestamp", "elapsed",                                 // Time
      "tx", "ty", "tz", "rx", "ry", "rz", "outlier",          // Information (translation, rotation, is outlier)
      "max_translation_m", "max_rotation_m", "history_size"   // Params
    ));
  }

  void TFOutlierFilter::setCurrentTransform(const Eigen::Affine3f &tf)
  {
    current_tf_ = tf;
    has_tf_ = true;
  }

  CloudPtr TFOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {

    // Set initial timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
    std::string timestamp = time_buf;

    CloudPtr result = std::make_shared<CloudT>();

    if (!has_tf_)
      return cloud;

    bool is_valid = isValid(current_tf_);

    if (is_valid)
    {
      if (tf_history_.size() >= history_size_)
        tf_history_.pop_front();
      tf_history_.push_back(current_tf_);
      result = cloud;
    } else {
      result = nullptr;
    }

    // Get elapsed time in milliseconds
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Compute translation and rotation
    Eigen::Vector3f translation = current_tf_.translation();
    Eigen::Vector3f rotation = current_tf_.rotation().eulerAngles(0, 1, 2); 

    // Build record for the current step
    auto& logger = io::Logger::get();
    logger.logStep(tag_, logger.makeRecord(
      timestamp,                    // Timestamp (ms since epoch)
      millis,                       // Elapsed time 
      translation.x(),              // tx
      translation.y(),              // ty
      translation.z(),              // tz
      rotation.x() * 180.0 / M_PI,  // Roll (x) in degrees
      rotation.y() * 180.0 / M_PI,  // Pitch (y) in degrees
      rotation.z() * 180.0 / M_PI,  // Yaw (z) in degrees
      !is_valid ? 1 : 0,            // outlier
      max_translation_m_,           // max_translation_m
      max_rotation_deg_,            // max_rotation_deg
      history_size_ 
    ));

    return result;
  
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
