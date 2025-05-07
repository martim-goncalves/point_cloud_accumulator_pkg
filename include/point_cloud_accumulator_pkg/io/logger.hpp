#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <iostream>
#include <string>
#include <sstream>
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
      static Logger &get();

      /** 
       * @brief Generic function using templates and variadic templates to return a tab-separated string.
       * @param args Column values to build the record with.
       * @return A TSV record.
       */
      template <typename... Args> 
      static std::string makeRecord(const Args&... args);
      
      /** 
       * @brief Sets the path for the run folder and creates it if missing.
       * @param folder Path to the runs folder.
       * @param run Name of the folder where the artifacts for a run of the same name are stored.
       */
      void setSaveFilePrefix(const std::string &folder, const std::string &run);

      /**
       * @brief Adds a record to the logs dictionary, identified by its destination file's tag as a key.
       * @param tag Key respective to the origin and destination of the lines.
       * @param record Record to append to the logs.
       */
      void logStep(const std::string &tag, const std::string &record);
      
      /**
       * @brief Flushes all of the logs to their respective log files. Logs are written to files under the run folder, 
       *        sharing the same name suffixed with the tag and `.tab` extension.
       */
      void flush();

    private:
      Logger();
      ~Logger();
      Logger(const Logger &) = delete;
      Logger &operator=(const Logger &) = delete;
      template <typename T> std::string toString(const T &value);

      /** Path to the folder where runs are saved. */
      std::string folder_; 
      /** Name of the folder where artifacts (e.g. maps, logs, etc.) for a run are kept. */
      std::string run_;
      /** Log lines separated by tag. */
      std::unordered_map<std::string, std::vector<std::string>> logs_;
      /** ... */
      std::mutex mutex_;

  };

} // namespace point_cloud_accumulator_pkg::io
