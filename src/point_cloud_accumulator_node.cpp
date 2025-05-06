#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <filesystem>
#include <logging.hpp>

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

      PointCloudAccumulatorNode() : Node("point_cloud_accumulator")
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
          -> setNext(std::make_shared<ColorConsistencyFilter>(t5, color_history_size_, saturation_thr_));

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

        // Setup unique save run folder
        std::time_t now = std::time(nullptr);
        char run_buf[100];
        std::strftime(run_buf, sizeof(run_buf), "%Y%m%d-%H%M%S", std::localtime(&now));
        std::string run_folder = savefolder_ + savefile_ + "_" + run_buf + "/";
        try 
        {
          std::filesystem::create_directories(run_folder);
          run_savefolder_ = run_folder;
        } catch (const std::filesystem::filesystem_error &e) {
          RCLCPP_ERROR(this->get_logger(), "Failed to create run folder '%s': %s", run_folder.c_str(), e.what());
        }

        // Create a save timer if periodic saving is enabled
        if (save_interval_seconds_ > 0) {
          save_timer_ = setInterval(save_interval_seconds_, [this]() { savePointCloud(); });
        }

        // Instantiate the step logger
        io::Logger::get().setSaveFilePrefix(savefolder_, savefile_);

      }

      ~PointCloudAccumulatorNode() 
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down...");
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

      CloudMsg handlePointCloud(const CloudMsg::SharedPtr &cloud)
      {
        // TODO Unmarshall message and process frame
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

      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;

      SubscriptionPtr topic_subscription_;
      PublisherPtr frame_publisher_;
      PublisherPtr accumulator_publisher_;
      TimerPtr save_timer_;

      int64_t min_points_thr_, max_points_thr_;
      double min_voxel_size_m_, max_voxel_size_m_;
      std::string run_savefolder_, savefolder_, savefile_;
      size_t save_interval_seconds_;

      double max_translation_m_, max_rotation_deg_;
      int tf_history_size_;

      int mean_k_, min_neighbors_, cloud_history_size_;
      float dist_thr_m_, min_appearance_ratio_;
      double std_ratio_;

      int saturation_thr_, color_history_size_;

      FilterPtr pipeline_;
      AccumulatorPtr accumulator_;

  };

  int main(int argc, char const *argv[])
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudAccumulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
  
}
