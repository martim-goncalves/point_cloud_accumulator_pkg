#include <rclcpp/rclcpp.hpp>

namespace point_cloud_accumulator_pkg
{

  class PointCloudAccumulatorNode : public rclcpp::Node
  {
    // TODO Implement main node logic here
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