#include "livox_pointcloud_converter/livox_to_pointcloud2.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace livox_pointcloud_converter
{

LivoxToPointCloud2Component::LivoxToPointCloud2Component(const rclcpp::NodeOptions & options)
: Node("livox_to_pointcloud2", options)
{
  // Declare parameters
  this->declare_parameter("input_topic", "/livox/lidar");
  this->declare_parameter("output_topic", "/livox/pointcloud2");
  this->declare_parameter("output_frame_id", "livox_frame");
  this->declare_parameter("use_intensity", true);
  this->declare_parameter("use_tag", false);
  this->declare_parameter("use_line", false);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  output_frame_id_ = this->get_parameter("output_frame_id").as_string();
  use_intensity_ = this->get_parameter("use_intensity").as_bool();
  use_tag_ = this->get_parameter("use_tag").as_bool();
  use_line_ = this->get_parameter("use_line").as_bool();

  // Create subscriber and publisher
  livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    input_topic_, 10,
    std::bind(&LivoxToPointCloud2Component::livoxMsgCallback, this, std::placeholders::_1));

  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, 10);

  RCLCPP_INFO(this->get_logger(), "Livox to PointCloud2 converter started");
  RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output frame: %s", output_frame_id_.c_str());
}

void LivoxToPointCloud2Component::livoxMsgCallback(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
  auto pointcloud2_msg = convertToPointCloud2(msg);
  pointcloud_pub_->publish(pointcloud2_msg);
}

sensor_msgs::msg::PointCloud2 LivoxToPointCloud2Component::convertToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr & livox_msg)
{
  // Create PCL point cloud
  pcl::PointCloud<LivoxPoint>::Ptr pcl_cloud(new pcl::PointCloud<LivoxPoint>);
  
  // Reserve space for efficiency
  pcl_cloud->points.reserve(livox_msg->point_num);
  
  // Convert each point
  for (const auto & point : livox_msg->points) {
    LivoxPoint pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    
    if (use_intensity_) {
      pcl_point.intensity = static_cast<float>(point.reflectivity);
    } else {
      pcl_point.intensity = 0.0f;
    }
    
    if (use_tag_) {
      pcl_point.tag = point.tag;
    } else {
      pcl_point.tag = 0;
    }
    
    if (use_line_) {
      pcl_point.line = point.line;
    } else {
      pcl_point.line = 0;
    }
    
    pcl_cloud->points.push_back(pcl_point);
  }
  
  // Set cloud properties
  pcl_cloud->width = livox_msg->point_num;
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = true;
  
  // Convert to ROS2 PointCloud2 message
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*pcl_cloud, output_msg);
  
  // Set header
  output_msg.header.stamp = livox_msg->header.stamp;
  output_msg.header.frame_id = output_frame_id_.empty() ? 
    livox_msg->header.frame_id : output_frame_id_;
  
  return output_msg;
}

}  // namespace livox_pointcloud_converter

RCLCPP_COMPONENTS_REGISTER_NODE(livox_pointcloud_converter::LivoxToPointCloud2Component)