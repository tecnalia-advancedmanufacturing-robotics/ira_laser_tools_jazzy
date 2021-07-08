#ifndef IRA_LASER_TOOLS_LADERSAN_MULTI_MERGER_HPP_
#define IRA_LASER_TOOLS_LADERSAN_MULTI_MERGER_HPP_

#include <iostream>
#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <deque>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer_interface.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"

#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "builtin_interfaces/msg/time.h"
#include "ira_laser_tools/CloudPile.hpp"

namespace laserscan_multi_merger
{
class LaserscanMerger : public rclcpp::Node
{
public:
  LaserscanMerger();
  int get_topic_index(std::string topic);
  void laserscan_topic_parser();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
  void laser_scan_to_cloud_deque(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    std::string topic);
  sensor_msgs::msg::LaserScan::UniquePtr pointcloud_to_laserscan(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
  sensor_msgs::msg::PointCloud2::SharedPtr laser_scan_to_pointcloud(
    const sensor_msgs::msg::LaserScan::SharedPtr scan);
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

private:
  void clear_cloud_stack();
  void pub_or_delete_clouds();
  void update_cloud_queue();
  void publish_latest_cloud_and_scan();
  bool is_scan_too_old(const builtin_interfaces::msg::Time stamp_time);
  int get_matching_pile(int topic_index, builtin_interfaces::msg::Time time_stamp);

  laser_geometry::LaserProjection projector_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;
  std::deque<CloudPile> cloud_deque;
  std::shared_ptr<rclcpp::Time> ref_time;
  std::vector<std::string> input_topics;
  std::vector<std::string> subscribed_topics;
  rclcpp::TimerBase::SharedPtr topic_parser_timer;

  bool allow_scan_delay;            // allow scan to be delayed (max_delay_time_sec)
  double max_delay_time_sec;        // max delay amount of a scan to node time
  double max_merge_time_diff_sec;   // max difference in scan time of merged scans

  double angle_min;
  double angle_max;
  double angle_increment;
  double time_increment;
  double scan_time;
  double range_min;
  double range_max;
  double inf_epsilon;
  bool use_inf;

  double min_height;
  double max_height;

  bool best_effort_enabled;

  std::string destination_frame;
  std::string cloud_destination_topic;
  std::string scan_destination_topic;
  std::string laserscan_topics;
};

}  // namespace laserscan_multi_merger

#endif  // IRA_LASER_TOOLS_LADERSAN_MULTI_MERGER_HPP_
