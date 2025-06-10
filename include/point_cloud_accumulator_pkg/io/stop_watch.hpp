#pragma once

#include <chrono>
#include <string>

using TimePoint = std::chrono::_V2::system_clock::time_point;

namespace point_cloud_accumulator_pkg::io
{
  class StopWatch
  {
    public:
      static StopWatch &get()
      {
        static StopWatch instance;
        return instance;
      }

      std::tuple<TimePoint, time_t> now() 
      { 
        auto start = std::chrono::system_clock::now();
        auto t0 = std::chrono::system_clock::to_time_t(start);
        return std::make_tuple(start, t0); 
      }

      std::string getTimestamp(time_t t)
      {
        char time_buf[100];
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d-%H%M%S", std::localtime(&t));
        std::string timestamp = time_buf;
        return timestamp;
      }

      int64_t getElapsedMillis(TimePoint start, TimePoint end)
      {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        return duration.count();
      }

      int64_t getElapsedMicros(TimePoint start, TimePoint end)
      {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        return duration.count();
      }

    private:
      StopWatch() = default;
      ~StopWatch() = default;
      StopWatch(const StopWatch &) = delete;
      StopWatch &operator=(const StopWatch &) = delete;
  };
} // namespace point_cloud_accumulator_pkg::io