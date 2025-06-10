#include "point_cloud_accumulator_pkg/filters/tf_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"
#include "point_cloud_accumulator_pkg/io/stop_watch.hpp"

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
    io::Logger::get().logStep(tag,
      "timestamp", "elapsed",                                 // Time
      "tx", "ty", "tz", "rx", "ry", "rz", "outlier",          // Information (translation, rotation, is outlier)
      "max_translation_m", "max_rotation_m", "history_size"   // Params
    );
  }

  void TFOutlierFilter::setCurrentTransform(const Eigen::Affine3f &tf)
  {
    current_tf_ = tf;
    has_tf_ = true;
  }

  CloudPtr TFOutlierFilter::applyFilter(const CloudPtr &cloud) const
  {
    // Set initial timestamp.
    auto& stopwatch = io::StopWatch::get();
    auto [start, t0] = stopwatch.now();

    // Guard against silliness.
    if (!has_tf_) return cloud;

    // Keep single exit point from here onwards.
    CloudPtr result = std::make_shared<CloudT>();

    // Push the current transform into the double sided queue and pop the oldest.
    if (tf_history_.size() >= history_size_)
    {
      tf_history_.pop_front();
    }
    tf_history_.push_back(current_tf_);
    
    // Check transform validity.
    bool is_valid = isValid(current_tf_);
    if (is_valid)
      result = cloud;

    // Compute translation and rotation
    Eigen::Vector3f translation = current_tf_.translation();
    Eigen::Vector3f rotation = current_tf_.rotation().eulerAngles(0, 1, 2); 

    // Build record for the current step
    auto [end, t] = stopwatch.now();
    auto& logger = io::Logger::get();
    logger.logStep(tag_,
      stopwatch.getTimestamp(t0),
      stopwatch.getElapsedMicros(start, end),  
      translation.x(),                          // tx
      translation.y(),                          // ty
      translation.z(),                          // tz
      rotation.x() * 180.0 / M_PI,              // Roll (x) in degrees
      rotation.y() * 180.0 / M_PI,              // Pitch (y) in degrees
      rotation.z() * 180.0 / M_PI,              // Yaw (z) in degrees
      !is_valid ? 1 : 0,                        // outlier
      max_translation_m_,                       // max_translation_m
      max_rotation_deg_,                        // max_rotation_deg
      history_size_ 
    );

    return result;
  }

  /**
   * @brief Validates if a transform is within the constraints set for the filter.
   * @param tf The transform being validated.
   * @return True if the transform is valid.
   */
  bool TFOutlierFilter::isValid(const Eigen::Affine3f &tf) const
  {
    // Assume first TF is good
    if (tf_history_.empty()) return true;

    // Compute mean of history
    Eigen::Vector3f mean_translation = Eigen::Vector3f::Zero();
    Eigen::Quaternionf mean_quat;
    std::vector<Eigen::Quaternionf> quats;
    for (const auto &tf : tf_history_) {
      mean_translation += tf.translation();
      quats.emplace_back(Eigen::Quaternionf(tf.rotation()));
    }
    mean_translation /= static_cast<float>(tf_history_.size());
    mean_quat = averageQuaternions(quats);
    
    // Compare current to mean (translation)
    float translation_mag = (tf.translation() - mean_translation).norm();

    // Compare current to mean (rotation)
    Eigen::Quaternionf current_quat(tf.rotation());
    float rotation_angle_rad = mean_quat.angularDistance(current_quat);
    float rotation_angle_deg = rotation_angle_rad * 180.0 / M_PI;

    return translation_mag <= max_translation_m_ && rotation_angle_deg <= max_rotation_deg_;
  }

  Eigen::Quaternionf TFOutlierFilter::averageQuaternions(const std::vector<Eigen::Quaternionf>& quats) const
  {
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero();
    for (const auto &q : quats) {
      Eigen::Vector4f vec(q.w(), q.x(), q.y(), q.z());
      A += vec * vec.transpose();
    }
    A /= quats.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(A);
    Eigen::Vector4f avg = solver.eigenvectors().col(3); // largest eigenvalue
    return Eigen::Quaternionf(avg(0), avg(1), avg(2), avg(3)).normalized();
  }

  
  // [TODO] | (Unused) :: Evaluate removal
  double TFOutlierFilter::getRotationAngleDeg(const Eigen::Matrix3f &a, const Eigen::Matrix3f &b) const
  {
    Eigen::Matrix3f delta = a.transpose() * b;
    double trace = std::max(-1.0f, std::min(1.0f, (delta.trace() - 1.0f) / 2.0f));
    double angle_rad = std::acos(trace);
    return angle_rad * 180.0 / M_PI;
  }

} // namespace point_cloud_accumulator_pkg::filters
