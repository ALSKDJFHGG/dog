#ifndef LIVOX_POINTCLOUD_CONVERTER__LIVOX_TO_POINTCLOUD2_HPP_
#define LIVOX_POINTCLOUD_CONVERTER__LIVOX_TO_POINTCLOUD2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace livox_pointcloud_converter
{

struct EIGEN_ALIGN16 LivoxPoint
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LivoxToPointCloud2Component : public rclcpp::Node
{
public:
  explicit LivoxToPointCloud2Component(const rclcpp::NodeOptions & options);

private:
  void livoxMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  
  sensor_msgs::msg::PointCloud2 convertToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr & livox_msg);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool use_intensity_;
  bool use_tag_;
  bool use_line_;
};

}  // namespace livox_pointcloud_converter

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_pointcloud_converter::LivoxPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint8_t, tag, tag)
                                  (std::uint8_t, line, line))

#endif  // LIVOX_POINTCLOUD_CONVERTER__LIVOX_TO_POINTCLOUD2_HPP_