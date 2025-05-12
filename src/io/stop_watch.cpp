#include "point_cloud_accumulator_pkg/io/stop_watch.hpp"

namespace point_cloud_accumulator_pkg::io
{

  StopWatch &StopWatch::get()
  {
    static StopWatch instance;
    return instance;
  }

  StopWatch::StopWatch() = default;
  StopWatch::~StopWatch() = default;

  /**
   * @brief Sets the initial timestamp.
   */
  void StopWatch::setStart()
  {
    start_ = std::chrono::system_clock::now();
    t0_ = std::chrono::system_clock::to_time_t(start_);
  }

  /**
   * @brief Converts the initial timestamp to a datetime string format (YMD-HMS).
   * @return Initial datetime (YMD-HMS).
   */
  std::string StopWatch::getTimestamp()
  {
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t0_));
    std::string timestamp = time_buf;
    return timestamp;
  }

  /**
   * @brief Gets the elapsed time since the initial timestamp.
   * @return Elapsed time in milliseconds.
   */
  int64_t StopWatch::getElapsedMillis()
  {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
    return duration.count();
  }

  int64_t StopWatch::getElapsedMicros()
  {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
    return duration.count();
  }


} // namespace point_cloud_accumulator_pkg::io
