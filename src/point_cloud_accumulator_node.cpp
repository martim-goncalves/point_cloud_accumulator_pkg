#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace std::chrono_literals;

class PointCloudAccumulatorNode : public rclcpp::Node
{

public:
  PointCloudAccumulatorNode() : Node("point_cloud_accumulator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    
    // Declare default topic names.
    std::string cloud_in = "/cloud_in";
    std::string cloud_frame = "/accumulator/cloud_frame";
    std::string cloud_out = "/accumulator/cloud_out";

    // Declare and get node parameters with default values.
    this->declare_parameter<int>("save_interval_seconds", 0);              // If 0 seconds, save only on shutdown.
    this->declare_parameter<std::string>("savefolder", "./artifacts/");    // Save folder path.
    this->declare_parameter<std::string>("savefile", "accumulated_cloud"); // Save file name.
    this->declare_parameter<int>("min_points_thr", 1'000'000);
    this->declare_parameter<int>("max_points_thr", 5'000'000);
    this->declare_parameter<double>("min_voxel_size_m", 0.025);            // Size for voxel downsampling.
    this->declare_parameter<double>("max_voxel_size_m", 0.1);              // Size for voxel downsampling.
    this->declare_parameter<int>("num_neighbors", 20);                     // For statistical outlier removal.
    this->declare_parameter<double>("std_ratio", 2.0);                     // For statistical outlier removal.

    save_interval_seconds_ = this->get_parameter("save_interval_seconds").as_int();
    savefolder_ = this->get_parameter("savefolder").as_string();
    savefile_ = this->get_parameter("savefile").as_string();
    min_points_thr_ = this->get_parameter("min_points_thr").as_int();
    max_points_thr_ = this->get_parameter("max_points_thr").as_int();
    min_voxel_size_m_ = this->get_parameter("min_voxel_size_m").as_double();
    max_voxel_size_m_ = this->get_parameter("max_voxel_size_m").as_double();
    voxel_size_m_ = min_voxel_size_m_;
    num_neighbors_ = this->get_parameter("num_neighbors").as_int();
    std_ratio_ = this->get_parameter("std_ratio").as_double();

    // Append trailling backslash to the save folder path.
    if (!savefolder_.empty() && savefolder_.back() != '/')
    {
      savefolder_ += "/";
    }

    // Subscriber for incoming point cloud frames.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_in,
        10,
        std::bind(&PointCloudAccumulatorNode::pointCloudCallback, this, std::placeholders::_1));

    // Publisher for the filtered point cloud frame.
    frame_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_frame, 10);

    // Publisher for the accumulated point cloud.
    accumulator_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out, 10);

    // Create a timer to periodically save the accumulated cloud if an interval is provided.
    if (save_interval_seconds_ > 0)
    {
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(save_interval_seconds_),
        std::bind(&PointCloudAccumulatorNode::timerCallback, this)
      );
    }

    // Log initialization.
    RCLCPP_INFO(
      this->get_logger(),
      "Point Cloud Accumulator node started. Saving every %d seconds.",
      save_interval_seconds_
    );

  }

  ~PointCloudAccumulatorNode()
  {
    RCLCPP_INFO(this->get_logger(), "Node shutting down; saving accumulated point cloud.");
    savePointCloud();
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    if (!msg)
    {
      RCLCPP_ERROR(this->get_logger(), "Received null point cloud message!");
      return;
    }

    // Attempt to get transform from 'msg->header.frame_id' to 'map'.
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      // Use a small timeout (adjust as needed).
      transform_stamped = tf_buffer_.lookupTransform(
          "map",                  // target frame
          msg->header.frame_id,   // source frame (camera)
          msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    // Transform the incoming cloud to the 'map' frame.
    sensor_msgs::msg::PointCloud2 cloud_in_map;
    tf2::doTransform(*msg, cloud_in_map, transform_stamped);

    // Convert ROS message to PCL cloud (stack-allocated).
    pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
    pcl::fromROSMsg(cloud_in_map, new_cloud);

    if (new_cloud.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
      return;
    }

    // Denoising using Statistical Outlier Removal (stack-allocated).
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(new_cloud)); // Pass as shared pointer.
    sor.setMeanK(num_neighbors_);
    sor.setStddevMulThresh(std_ratio_);
    sor.filter(filtered_cloud);

    // [NOTE] :: Publish downsampled clouds and ingest them with the octomap node to account for occupancy and update
    //           terrain as the mapper gets closer (far surfaces have more uncertainty).

    // Convert filtered frame back to ROS message and publish.
    publishPointCloud(frame_publisher_, filtered_cloud, cloud_in_map.header.stamp, "map");

    // Accumulate points.
    accumulated_cloud_ += filtered_cloud;

    // Downsampling using Voxel Grid (stack-allocated).
    pcl::PointCloud<pcl::PointXYZRGB> downsampled_cloud;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(accumulated_cloud_));
    voxel.setLeafSize(voxel_size_m_, voxel_size_m_, voxel_size_m_);
    voxel.filter(downsampled_cloud);

    // Replace accumulated cloud.
    accumulated_cloud_ = downsampled_cloud;

    // Convert accumulator back to ROS message and publish.
    publishPointCloud(accumulator_publisher_, downsampled_cloud, cloud_in_map.header.stamp, "map");

    // Update voxel size.
    double new_voxel_size_m = getAdaptiveVoxelSize(
      accumulated_cloud_.size(), 
      min_points_thr_, 
      max_points_thr_, 
      min_voxel_size_m_, 
      max_voxel_size_m_
    );
    if (new_voxel_size_m > voxel_size_m_) 
    {
      voxel_size_m_ = new_voxel_size_m;
    }

    RCLCPP_DEBUG(
      this->get_logger(), 
      "Accumulated cloud updated and published :: %ldpts %fmm.", 
      accumulated_cloud_.size(), 
      voxel_size_m_
    );

  }

  void timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer triggered: saving accumulated point cloud.");
    savePointCloud();
  }

  void publishPointCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher, 
    const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud,
    const rclcpp::Time& stamp,
    const std::string& frame_id
  ) {

    if (point_cloud.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Attempted to publish an empty point cloud.");
      return;
    }

    // Convert point cloud to ROS message.
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(point_cloud, output_msg);
    output_msg.header.stamp = stamp;
    output_msg.header.frame_id = frame_id;

    // Publish message.
    publisher->publish(output_msg);

  }

  void savePointCloud()
  {

    if (accumulated_cloud_.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No points to save.");
      return;
    }

    // Generate a filename with a timestamp.
    std::time_t now = std::time(nullptr);
    char time_buf[100];
    std::strftime(time_buf, sizeof(time_buf), "%Y%m%d-%H%M%S", std::localtime(&now));

    // Get cloud voxel resolution in mm.
    std::string res_mm = std::to_string(int(voxel_size_m_ * 1000)) + "mm";

    // Get number of accumulated points.
    std::string acc_pts = std::to_string(accumulated_cloud_.size()) + "pts";

    // Determine file extension.
    std::string file_extension = ".ply"; // Default file extension.
    size_t dot_pos = savefile_.find_last_of(".");
    if (dot_pos != std::string::npos)
    {
      file_extension = savefile_.substr(dot_pos); // Get provided extension.
    }

    // Remove extension from filename if present.
    std::string base_name = (dot_pos != std::string::npos) ? savefile_.substr(0, dot_pos) : savefile_;

    // Construct final filename.
    std::string final_filename; 
    final_filename.append(std::string(time_buf) + "_");
    final_filename.append(base_name + "_");
    final_filename.append(res_mm + "_");
    final_filename.append(acc_pts);
    final_filename.append(file_extension);
    std::string final_savepath = savefolder_ + final_filename;

    // Save the accumulated cloud to a file.
    if (pcl::io::savePLYFileBinary(final_savepath, accumulated_cloud_) == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Saved accumulated point cloud to %s", final_savepath.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to %s", final_savepath.c_str());
    }

  }

  /**
   * Dynamically calculates a voxel size adapted to the current number of points. Interpolates the result between a 
   * minimum and a maximum size using a logistic sigmoid function applied between two point number thresholds. 
   * @param num_points Current number of accumulated points.
   * @param min_points Minimum point threshold to start voxel size interpolation.
   * @param max_points Maximum point threshold to end voxel size interpolation.
   * @param min_voxel_size The minimum size of a voxel (higher resolution).
   * @param max_voxel_size The maximum size of a voxel (lower resolution).
   * @return Current voxel size in arbitrary units.
   */
  double getAdaptiveVoxelSize(
    size_t num_points, 
    size_t min_points, 
    size_t max_points, 
    double min_voxel_size, 
    double max_voxel_size
  ) {

    // [Guard Clause] :: Avoid divide-by-zero.
    if (max_points <= min_points) { return min_voxel_size; }

    // Clamp number of points within the range.
    size_t clamped_points = std::min(std::max(num_points, min_points), max_points);

    // Find logistic sigmoid function to determine voxel size in metres.
    double midpoint = (min_points + max_points) / 2.0;
    double k = 10.0 / (max_points - min_points);
    double s = 1.0 / (1.0 + std::exp(-k * (clamped_points - midpoint)));

    // Interpolate voxel size between min and max.
    return min_voxel_size + s * (max_voxel_size - min_voxel_size);

  }

  // Private member variables (attributes).
  int save_interval_seconds_;
  std::string savefolder_;
  std::string savefile_;
  int64_t min_points_thr_;
  int64_t max_points_thr_;
  double voxel_size_m_;
  double min_voxel_size_m_;
  double max_voxel_size_m_;
  int num_neighbors_;
  double std_ratio_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulator_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  pcl::PointCloud<pcl::PointXYZRGB> accumulated_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudAccumulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
