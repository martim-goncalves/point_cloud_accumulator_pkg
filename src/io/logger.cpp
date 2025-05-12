#include <fstream>
#include <filesystem>

#include "point_cloud_accumulator_pkg/io/logger.hpp"

namespace point_cloud_accumulator_pkg::io
{

  Logger &Logger::get()
  {
    static Logger instance;
    return instance;
  }

  Logger::Logger() = default;
  Logger::~Logger() = default;

  void Logger::setEnabled(bool enabled) { enabled_ = enabled; }

  void Logger::setSaveFilePrefix(const std::string &folder, const std::string &run)
  {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    folder_ = folder; // e.g. ./artifacts/YMD-HMS_accum_cloud/
    run_ = run;       // e.g. YMD-HMS_accum_cloud
    std::filesystem::create_directories(folder);
  }

  void Logger::logStep(const std::string &tag, const std::string &record)
  {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::string artifact_name = run_ + "_" + tag;
    std::string filename = folder_ + artifact_name + ".tab";
    std::ofstream out(filename, std::ios::app);
    if (out.is_open()) out << record << '\n';
  }

} // namespace point_cloud_accumulator_pkg::io
