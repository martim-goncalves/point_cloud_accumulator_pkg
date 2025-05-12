#pragma once

#include <chrono>
#include <string>

namespace point_cloud_accumulator_pkg::io
{
  class StopWatch
  {
    public:
      using TimePoint = std::chrono::_V2::system_clock::time_point;
      static StopWatch &get();
      void setStart();
      std::string getTimestamp();
      int64_t getElapsedMillis();
      int64_t getElapsedMicros();

  private:
      StopWatch();
      ~StopWatch();
      StopWatch(const StopWatch &) = delete;
      StopWatch &operator=(const StopWatch &) = delete;

      TimePoint start_;
      time_t t0_;
  };
} // namespace point_cloud_accumulator_pkg::io
