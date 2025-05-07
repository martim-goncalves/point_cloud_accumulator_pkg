#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <filesystem>

#include "point_cloud_accumulator_pkg/curves/logistic_sigmoid.hpp"
#include "point_cloud_accumulator_pkg/voxel_scaler.hpp"
#include "point_cloud_accumulator_pkg/filters/spatial_filter.hpp"
#include "point_cloud_accumulator_pkg/filters/statistical_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/filters/temporal_filter.hpp"
#include "point_cloud_accumulator_pkg/filters/tf_outlier_filter.hpp"
#include "point_cloud_accumulator_pkg/filters/color_consistency_filter.hpp"
#include "point_cloud_accumulator_pkg/accumulator.hpp"
#include "point_cloud_accumulator_pkg/io/logger.hpp"

// Type length would be bizarre without these!
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using TimerPtr = rclcpp::TimerBase::SharedPtr;
using PublisherPtr = rclcpp::Publisher<CloudMsg>::SharedPtr;
using SubscriptionPtr = rclcpp::Subscription<CloudMsg>::SharedPtr;

namespace point_cloud_accumulator_pkg
{

  class PointCloudAccumulatorNode : public rclcpp::Node
  {

    public:

      using FilterPtr = std::shared_ptr<Filter>;
      using AccumulatorPtr = std::unique_ptr<Accumulator>;

      PointCloudAccumulatorNode() 
        : Node("point_cloud_accumulator")
        , tf_buffer_(this->get_clock())
        , tf_listener_(tf_buffer_)
      {
        // Declare default topic names
        std::string cloud_in = "/cloud_in";
        std::string cloud_frame = "/accumulator/cloud_frame";
        std::string cloud_out = "/accumulator/cloud_out";

        // Declare standard parameters
        this->declare_parameter<int>("min_points_thr", 1'000'000);              // Downsampling starts.
        this->declare_parameter<int>("max_points_thr", 5'000'000);              // Downsampling ends.
        this->declare_parameter<double>("min_voxel_size_m", 0.025);             // High resolution.
        this->declare_parameter<double>("max_voxel_size_m", 0.1);               // Low resolution.
        this->declare_parameter<std::string>("savefolder", "./artifacts/");     // Save folder path.
        this->declare_parameter<std::string>("savefile", "accumulated_cloud");  // Run name.
        this->declare_parameter<int>("save_interval_seconds", 0);               // If zero, save on shutdown only.
        this->declare_parameter<int>("log_interval_seconds", 30);               // If zero, log on shutdown only.

        // Declare outlier transform filter parameters
        this->declare_parameter<double>("filters.transform.max_translation_m", 0.3);
        this->declare_parameter<double>("filters.transform.max_rotation_deg", 15.0);
        this->declare_parameter<int>("filters.transform.history_size", 5);

        // Declare outlier point filter parameters
        this->declare_parameter<int>("filters.point.num_neighbors", 20);              // SOR
        this->declare_parameter<float>("filters.point.dist_thr_m", 0.02f);            // Temporal Spatial
        this->declare_parameter<float>("filters.point.min_appearance_ratio", 0.6f);   // Temporal
        this->declare_parameter<int>("filters.point.min_neighbors", 5);               // Spatial
        this->declare_parameter<double>("filters.point.std_ratio", 2.0);              // SOR
        this->declare_parameter<int>("filters.point.history_size", 5);                // Temporal

        // Declare color consistency filter parameters
        this->declare_parameter<int>("filters.color.saturation_thr", 240);
        this->declare_parameter<int>("filters.color.history_size", 10);

        // Get general parameter values
        this->get_parameter("min_points_thr", min_points_thr_);
        this->get_parameter("max_points_thr", max_points_thr_);
        this->get_parameter("min_voxel_size_m", min_voxel_size_m_);
        this->get_parameter("max_voxel_size_m", max_voxel_size_m_);
        this->get_parameter("savefolder", savefolder_);
        this->get_parameter("savefile", savefile_);
        this->get_parameter("save_interval_seconds", save_interval_seconds_);
        this->get_parameter("log_interval_seconds", log_interval_seconds_);

        // Get outlier transform filter parameters
        this->get_parameter("filters.transform.max_translation_m", max_translation_m_);
        this->get_parameter("filters.transform.max_rotation_deg", max_rotation_deg_);
        this->get_parameter("filters.transform.history_size", tf_history_size_);

        // Get outlier point filter parameters
        this->get_parameter("filters.point.num_neighbors", mean_k_);
        this->get_parameter("filters.point.dist_thr_m", dist_thr_m_);
        this->get_parameter("filters.point.min_appearance_ratio", min_appearance_ratio_);
        this->get_parameter("filters.point.min_neighbors", min_neighbors_);
        this->get_parameter("filters.point.std_ratio", std_ratio_);
        this->get_parameter("filters.point.history_size", cloud_history_size_);

        // Get color consistency filter parameters
        this->get_parameter("filters.color.saturation_thr", saturation_thr_);
        this->get_parameter("filters.color.history_size", color_history_size_);

        // Setup unique save run folder (e.g. ./artifacts/run_name/ where "run_name" is "YMD-HMS_savefile")
        std::time_t now = std::time(nullptr);
        char time_buf[100];
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d-%H%M%S", std::localtime(&now));
        std::string run_name = std::string(time_buf) + "_" + savefile_;   // e.g. YMD-HMS_accum_cloud
        std::string run_folder = savefolder_ + run_name + "/";            // e.g. ./artifacts/run_name/
        try 
        {
          std::filesystem::create_directories(run_folder);
          run_savefolder_ = run_folder;
        } catch (const std::filesystem::filesystem_error &e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to create run folder '%s': %s", run_folder.c_str(), e.what());
        }

        // Instantiate the step logger
        io::Logger::get().setSaveFilePrefix(run_folder, run_name);

        using namespace point_cloud_accumulator_pkg::filters;
        using namespace point_cloud_accumulator_pkg::curves;

        // Filter tags
        std::string t1 = "tfoutlier", t2 = "spatial", t3 = "sor", t4 = "temporal", t5 = "color";

        // Starting filter in the pipeline
        pipeline_ = std::make_shared<TFOutlierFilter>(t1, max_translation_m_, max_rotation_deg_, tf_history_size_);

        // Chain filters
        pipeline_ // TF -> Spatial -> SOR -> Temporal -> Color
          -> setNext(std::make_shared<SpatialFilter>(t2, dist_thr_m_, min_neighbors_))
          -> setNext(std::make_shared<StatisticalOutlierFilter>(t3, mean_k_, std_ratio_))
          -> setNext(std::make_shared<TemporalFilter>(t4, cloud_history_size_, dist_thr_m_, min_appearance_ratio_))
          // -> setNext(std::make_shared<ColorConsistencyFilter>(t5, color_history_size_, saturation_thr_))
        ;

        // Create interpolation curve for adaptive voxel size
        auto curve = std::make_unique<LogisticSigmoid>(
          min_points_thr_, max_points_thr_, min_voxel_size_m_, max_voxel_size_m_
        );

        // Create the voxel scaler
        auto scaler = std::make_shared<VoxelScaler>(
          min_points_thr_, max_points_thr_, min_voxel_size_m_, max_voxel_size_m_, std::move(curve)
        );

        // Inject the filter pipeline and voxel scaler into the point cloud accumulator
        accumulator_ = std::make_unique<Accumulator>(min_voxel_size_m_, pipeline_, scaler);

        // ...
        topic_subscription_ = this->create_subscription<CloudMsg>(
          cloud_in, 10,
          std::bind(&PointCloudAccumulatorNode::handlePointCloud, this, std::placeholders::_1)
        );

        // ...
        frame_publisher_ = this->create_publisher<CloudMsg>(cloud_frame, 10);
        accumulator_publisher_ = this->create_publisher<CloudMsg>(cloud_out, 10);

        // Create a save timer if periodic saving is enabled
        if (save_interval_seconds_ > 0)
          save_timer_ = setInterval(
            save_interval_seconds_, std::bind(&PointCloudAccumulatorNode::savePointCloud, this)
          );

        // Create a log timer to keep the RAM from getting exhausted
        if (log_interval_seconds_ > 0)
          log_timer_ = setInterval(
            log_interval_seconds_, std::bind(&PointCloudAccumulatorNode::logRecords, this)
          );

        // Log starting configuration
        RCLCPP_INFO(this->get_logger(),
          "[+] PointCloudAccumulatorNode initialized:\n"
          "\t - Run artifacts folder: %s\n"
          "\t - Run name: %s\n"
          "\t - Min/max point threshold: %d / %d\n"
          "\t - Min/max voxel size: %.4f / %.4f m\n"
          "\t - TF filter: max_translation = %.2f m, max_rotation = %.2f deg, history = %d\n"
          "\t - Spatial filter: dist_thr = %.4f m, min_neighbors = %d\n"
          "\t - SOR filter: mean_k = %d, std_ratio = %.2f\n"
          "\t - Temporal filter: history = %d, dist_thr = %.4f m, min_ratio = %.2f\n"
          "\t - Color filter: history = %d, sat_thr = %d\n"
          "\t - Save interval: %d s | Log interval: %d s",
          run_savefolder_.c_str(), run_name.c_str(),
          min_points_thr_, max_points_thr_,
          min_voxel_size_m_, max_voxel_size_m_,
          max_translation_m_, max_rotation_deg_, tf_history_size_,
          dist_thr_m_, min_neighbors_,
          mean_k_, std_ratio_,
          cloud_history_size_, dist_thr_m_, min_appearance_ratio_,
          color_history_size_, saturation_thr_,
          save_interval_seconds_, log_interval_seconds_
        );
      }

      ~PointCloudAccumulatorNode() 
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down...");
        logRecords();
        savePointCloud();
      }

    private:

      /**
       * @brief Instantiate timer to invoke the callback at a set interval.
       * @param seconds Time interval, in seconds.
       * @param callback A callback function activated every interval.
       * @return A pointer to the wall timer.
       */
      TimerPtr setInterval(size_t seconds, std::function<void()> callback)
      {
        return this->create_wall_timer(
          std::chrono::seconds(seconds), [callback]() { callback(); });
      }

      void handlePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
      {
        // Extract point cloud from ROS 2 message
        CloudPtr cloud = std::make_shared<CloudT>();
        pcl::fromROSMsg(*msg, *cloud);

        // Skip empty input early
        if (cloud->empty()) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, 
            "Received an empty input point cloud, skipping."
          );
          return;
        }

        // Attempt to lookup transform from cloud frame to message frame
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        try 
        {
          geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_.lookupTransform(
            "map", 
            msg->header.frame_id, 
            msg->header.stamp, 
            tf2::durationFromSec(0.1)
          );
          tf = tf2::transformToEigen(tf_stamped).cast<float>();
          pipeline_->setCurrentTransform(tf);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, 
            "TF lookup failed: %s", ex.what()
          );
        }

        // Ingest and filter the cloud
        CloudPtr filtered = accumulator_->ingest(cloud);
        CloudPtr accumulated = accumulator_->getAccumulatedCloud();

        // Skip publishing empty frames
        if (!filtered || filtered->empty()) 
        {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, 
            "Filtered cloud is empty, skipping publish."
          );
          return;
        }

        // Publish filtered and accumulated point clouds
        publishPointCloud(frame_publisher_, *filtered, msg->header.stamp, "map");
        publishPointCloud(accumulator_publisher_, *accumulated, msg->header.stamp, "map");

      }

      /**
       * @brief Convert a point cloud to a ROS 2 message and publish it.
       * @param publisher A pointer to the ROS 2 publisher.
       * @param cloud The point cloud to publish.
       * @param stamp The message timestamp. 
       * @param frame_id The message's frame ID. 
       */ 
      void publishPointCloud(
        const PublisherPtr &publisher, 
        const CloudT &cloud, 
        const rclcpp::Time &stamp, 
        const std::string &frame_id
      ) {
        if (cloud.empty())
        {
          RCLCPP_WARN(this->get_logger(), "Attempted to publish an empty point cloud!");
          return;
        }

        // Convert point cloud to ROS message.
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header.stamp = stamp;
        output_msg.header.frame_id = frame_id;

        // Publish message.
        publisher->publish(output_msg);
      }

      void savePointCloud()
      {
        auto cloud = accumulator_->getAccumulatedCloud();
        if (!cloud || cloud->empty()) {
          RCLCPP_WARN(this->get_logger(), "No data to save");
          return;
        }

        std::time_t now = std::time(nullptr);
        char time_buf[100];
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d-%H%M%S", std::localtime(&now));

        std::string res_mm = std::to_string(int(min_voxel_size_m_ * 1000)) + "mm";
        std::string acc_pts = std::to_string(cloud->size()) + "pts";

        std::string file_extension = ".ply";
        size_t dot_pos = savefile_.find_last_of(".");
        if (dot_pos != std::string::npos) {
          file_extension = savefile_.substr(dot_pos);
        }
        std::string base_name = (dot_pos != std::string::npos) ? savefile_.substr(0, dot_pos) : savefile_;

        std::string final_filename = std::string(time_buf) + "_" + base_name + "_" + res_mm + "_" + acc_pts + file_extension;
        std::string final_savepath = run_savefolder_ + final_filename;

        if (pcl::io::savePLYFileBinary(final_savepath, *cloud) == 0) {
          RCLCPP_INFO(this->get_logger(), "Saved accumulated point cloud to %s", final_savepath.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", final_savepath.c_str());
        }
      }

      void logRecords()
      {
        io::Logger::get().flush();
        RCLCPP_INFO(this->get_logger(), "Saved run logs to disk.");
      }

      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;

      SubscriptionPtr topic_subscription_;
      PublisherPtr frame_publisher_;
      PublisherPtr accumulator_publisher_;
      TimerPtr save_timer_;
      TimerPtr log_timer_;

      int64_t min_points_thr_, max_points_thr_;
      double min_voxel_size_m_, max_voxel_size_m_;
      std::string run_savefolder_, savefolder_, savefile_;
      size_t save_interval_seconds_, log_interval_seconds_;

      double max_translation_m_, max_rotation_deg_;
      int tf_history_size_;

      int mean_k_, min_neighbors_, cloud_history_size_;
      float dist_thr_m_, min_appearance_ratio_;
      double std_ratio_;

      int saturation_thr_, color_history_size_;

      std::shared_ptr<filters::TFOutlierFilter> pipeline_;
      AccumulatorPtr accumulator_;

  };
  
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<point_cloud_accumulator_pkg::PointCloudAccumulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
