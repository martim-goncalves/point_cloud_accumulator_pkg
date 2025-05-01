#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions.h>

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
        this->declare_parameter<std::string>("savefile", "accumulated_cloud");  // Save file name.
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

        // TODO Processing system initialization
        // 1. Create a Curve
        // 2. Create a VoxelScaler
        // 3. Create a Filter pipeline
        // 4. Create the Accumulator

      }

      ~PointCloudAccumulatorNode() 
      {
        RCLCPP_INFO(this->get_logger(), "Node shutting down...");
        savePointCloud();
      }

    private:

      TimerPtr setInterval(size_t seconds, std::function<void()> callback)
      {
        // TODO Instantiate timer to invoke the callback at the set interval
      }

      CloudMsg handlePointCloud(const CloudMsg::SharedPtr &cloud)
      {
        // TODO Unmarshall message and process frame
      }

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
        // TODO Implement save to file logic
      }

      tf2_ros::Buffer tf_buffer_;
      tf2_ros::TransformListener tf_listener_;

      SubscriptionPtr topic_subscription_;
      PublisherPtr frame_publisher_;
      PublisherPtr accumulator_publisher_;
      TimerPtr save_timer_;

      int64_t min_points_thr_, max_points_thr_;
      double min_voxel_size_m_, max_voxel_size_m_;
      std::string savefolder_, savefile_;
      size_t save_interval_seconds_;

      double max_translation_m_, max_rotation_deg_;
      int tf_history_size_;

      int mean_k_, min_neighbors_, cloud_history_size_;
      float dist_thr_m_, min_appearance_ratio_;
      double std_ratio_;

      int saturation_thr_, color_history_size_;

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
