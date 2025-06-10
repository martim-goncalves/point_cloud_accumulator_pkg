#pragma once

#include <string>
#include <filesystem>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <type_traits>

namespace point_cloud_accumulator_pkg::io
{
  class Logger
  {
    public:
      /**
       * @brief Create or retrieve an existing Logger instance.
       * @return Logger instance.
       */
      static Logger &get()
      {
        static Logger instance;
        return instance;
      }

      void setEnabled(bool enabled) 
      {
        enabled_ = enabled;
      }

      /**
       * @brief Sets the path for the run's artifacts folder and creates it if missing.
       * @param folder Path to the folder containing the artifacts for a certain run.
       * @param run Name of the folder where the artifacts for a run of the same name are stored.
       */
      void setSaveFilePrefix(const std::string &folder, const std::string &run)
      {
        if (!enabled_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        folder_ = folder; // e.g. ./artifacts/YMD-HMS_run_name/
        run_ = run;       // e.g. YMD-HMS_run_name
        std::filesystem::create_directories(folder);
      }

      /**
       * @brief Logs a processing step as a record to a file identified by its orign tag.
       * @param tag Tag representing the source or origin of the record being logged.
       * @param record Record to append to the logs.
       */
      template <typename... Args>
      void logStep(const std::string &tag, const Args &...record)
      {
        if (!enabled_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        std::string artifact_name = run_ + "_" + tag;
        std::string filename = folder_ + artifact_name + ".tab";
        std::ofstream out(filename, std::ios::app);
        if (out.is_open()) out << makeRecord(record...) << '\n';
      }

    private:
      /** Path to the folder where run artifacts are saved. */
      std::string folder_;
      /** Name of the folder where artifacts (e.g. maps, logs, etc.) for a run are kept. */
      std::string run_;
      /** ... */
      std::mutex mutex_;
      /** ... */
      bool enabled_;

      Logger() = default;
      ~Logger() = default;
      Logger(const Logger &) = delete;
      Logger &operator=(const Logger &) = delete;

      template <typename T>
      static std::string toString(const T &value)
      {
        std::stringstream ss;
        ss << value;
        return ss.str();
      }

      /**
       * @brief Generic function using templates and variadic templates to return a tab-separated string.
       * @param args Column values to build the record with.
       * @return A TSV record.
       */
      template <typename... Args>
      static std::string makeRecord(const Args &...args)
      {
        std::stringstream ss;
        size_t n = sizeof...(Args);
        size_t i = 0;
        auto append_with_tab = [&](const auto &arg)
        {
          ss << toString(arg);
          if (++i < n)
          {
            ss << '\t';
          }
        };
        (append_with_tab(args), ...);
        return ss.str();
      }
  };
} // namespace point_cloud_accumulator_pkg::io
