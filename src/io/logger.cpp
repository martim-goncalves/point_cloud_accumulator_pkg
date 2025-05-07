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

  void Logger::setSaveFilePrefix(const std::string &folder, const std::string &run)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    folder_ = folder;
    run_ = run;
		std::string run_folder = folder + "/" + run + "/"; 
    std::filesystem::create_directories(run_folder);
  }

  void Logger::logStep(const std::string &filter_tag, const std::string &record)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    logs_[filter_tag].push_back(record);
  }

  void Logger::flush()
  {

		// Lock
    std::lock_guard<std::mutex> lock(mutex_);

		// For each tag-line pair, attempt to flush the logs to a file.
    for (auto &[tag, records] : logs_)
    {

			// [Guard Clause] :: Skip to next if no lines to log for a given tag.
      if (records.empty()) continue;

			// Build the save destination's file path
			std::string run_folder = folder_ + "/" + run_;
			std::string artifact_name = run_ + "_" + tag;
      std::string filename = run_folder + "/" + artifact_name + ".tab";
      std::ofstream out(filename, std::ios::app);

			// [Guard Clause] :: Skip to next if output failed to open.
      if (!out.is_open()) continue;

			// Append each line to the output file.
      for (const auto &record : records)
      {
        out << record;
      }

			// Clear the lines from the Logger.
      records.clear();

    }

  }

} // namespace point_cloud_accumulator_pkg::io
