#include "ira_laser_tools/laserscan_multi_merger.hpp"

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <map>
#include <vector>
#include <utility>

namespace laserscan_multi_merger
{
LaserscanMerger::LaserscanMerger()
: Node("laser_multi_merger")
{
  destination_frame = this->declare_parameter<std::string>("destination_frame", "dest_frame");
  cloud_destination_topic = this->declare_parameter<std::string>(
    "cloud_destination_topic",
    "/merged_cloud");
  scan_destination_topic = this->declare_parameter<std::string>(
    "scan_destination_topic",
    "/merged_scan");
  laserscan_topics = this->declare_parameter<std::string>("laserscan_topics", "");
  min_height = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min = this->declare_parameter("angle_min", -M_PI);
  angle_max = this->declare_parameter("angle_max", M_PI);
  angle_increment = this->declare_parameter("angle_increment", M_PI / 180.0);
  time_increment = this->declare_parameter("time_increment", 0);
  scan_time = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min = this->declare_parameter("range_min", 0.0);
  range_max = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  use_inf = this->declare_parameter("use_inf", true);
  inf_epsilon = this->declare_parameter("inf_epsilon", 1.0);

  topic_parser_timer = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&LaserscanMerger::laserscan_topic_parser, this));

  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

  point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->cloud_destination_topic.c_str(), 1);
  laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    this->scan_destination_topic.c_str(), 1);
}

void LaserscanMerger::laserscan_topic_parser()
{
  // Necessity for sleep workaround to get topics
  // https://github.com/ros2/ros2/issues/1057
  std::map<std::string, std::vector<std::string>> topics;
  uint r = 0;
  while (topics.size() < 3) { // parameter_events & rosout
    topics = this->get_topic_names_and_types();  // first:name, second:type
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (++r > 5) {
      RCLCPP_ERROR(this->get_logger(), "No published topics found!");
      return;
    }
  }

  std::vector<std::string> published_scan_topics;
  for (auto const & topic : topics) {
    if (topic.second[0].compare("sensor_msgs/msg/LaserScan") == 0) {
      std::string topic_name = topic.first;
      if (topic_name.at(0) == '/') {
        topic_name = topic_name.substr(1);
      }
      published_scan_topics.push_back(topic_name);
    }
  }

  std::istringstream iss(this->laserscan_topics);
  std::vector<std::string> tokens((std::istream_iterator<std::string>(
      iss)), std::istream_iterator<std::string>());
  std::vector<std::string> tmp_input_topics;

  // make sure missing topics are published LaserScan topics
  for (auto const & token : tokens) {
    if (std::find(
        subscribed_topics.begin(), subscribed_topics.end(),
        token) == subscribed_topics.end())
    {
      if (std::find(
          published_scan_topics.begin(), published_scan_topics.end(),
          token) != published_scan_topics.end())
      {
        tmp_input_topics.push_back(token);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Topic %s [sensor_msg/LaserScan] does not seem to be published yet. Could not subscribe.",
          token.c_str());
      }
    }
  }

  // clean up duplicate topics
  std::sort(tmp_input_topics.begin(), tmp_input_topics.end());
  std::vector<std::string>::iterator last =
    std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
  tmp_input_topics.erase(last, tmp_input_topics.end());
  input_topics = tmp_input_topics;
  
  // Create subscriptions
  if (input_topics.size() > 0) {
    scan_subscribers.resize(input_topics.size());
    clouds_modified.resize(input_topics.size());
    clouds.resize(input_topics.size());
    std::stringstream output_info;
    std::copy(
      input_topics.begin(), input_topics.end(),
      std::ostream_iterator<std::string>(output_info, " "));
    RCLCPP_INFO(
      this->get_logger(),
      "Subscribing to %ld topics.", scan_subscribers.size());
    for (uint i = 0; i < input_topics.size(); ++i) {
      // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
      std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback =
        std::bind(
        &LaserscanMerger::scanCallback,
        this, std::placeholders::_1, input_topics[i]);
      scan_subscribers[i] = this->create_subscription<
        sensor_msgs::msg::LaserScan>(input_topics[i].c_str(), 1, callback);
      RCLCPP_INFO(
        this->get_logger(),
        "Subscribed to %s.", input_topics[i].c_str());
      clouds_modified[i] = false;
      subscribed_topics.push_back(input_topics[i]);
    }
  }
}

void LaserscanMerger::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan,
  std::string topic)
{
  RCLCPP_DEBUG(
  this->get_logger(),
  "Scan callback of %s", topic.c_str());

  sensor_msgs::msg::PointCloud2 singleScanCloud;

  // transform scan if necessary
  // if (scan->header.frame_id != destination_frame)
  tf2::TimePoint time_point = tf2_ros::fromMsg(scan->header.stamp);
  // std::string *err_msg;
  bool canTransform = tf_buffer_->canTransform(
    destination_frame.c_str(), scan->header.frame_id.c_str(), time_point,
    tf2::durationFromSec(1.0));
  if (!canTransform) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not look up transform from %s to %s at time [%ld.%ld]",
      scan->header.frame_id.c_str(), destination_frame.c_str(),
      scan->header.stamp.sec, scan->header.stamp.nanosec);
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Transform available");
  }
  projector_.transformLaserScanToPointCloud(
    destination_frame.c_str(), *scan, singleScanCloud, *tf_buffer_);

  for (uint i = 0; i < input_topics.size(); ++i) {
    if (topic.compare(input_topics[i]) == 0) {
      pcl_conversions::toPCL(singleScanCloud, clouds[i]);
      clouds_modified[i] = true;
    }
  }

  // Count how many scans we have
  uint totalClouds = 0;
  for (uint i = 0; i < clouds_modified.size(); ++i) {
    if (clouds_modified[i]) {
      ++totalClouds;
    }
  }

  // Go ahead only if all subscribed scans have arrived
  if (totalClouds == clouds_modified.size()) {
    pcl::PCLPointCloud2 merged_cloud = clouds[0];
    clouds_modified[0] = false;

    for (uint i = 1; i < clouds_modified.size(); ++i) {
      pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
      clouds_modified[i] = false;
    }

    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);
    this->point_cloud_publisher_->publish(*cloud_msg);

    pointcloud_to_laserscan(cloud_msg);
  }
}

void LaserscanMerger::pointcloud_to_laserscan(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  // https://github.com/ros-perception/pointcloud_to_laserscan/blob/554173b73f7b5fd2e4d9218b1096cb4c7dcbae1c/src/pointcloud_to_laserscan_node.cpp#L137

  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  scan_msg->angle_min = this->angle_min;
  scan_msg->angle_max = this->angle_max;
  scan_msg->angle_increment = this->angle_increment;
  scan_msg->time_increment = this->time_increment;
  scan_msg->scan_time = this->scan_time;
  scan_msg->range_min = this->range_min;
  scan_msg->range_max = this->range_max;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  if (this->use_inf) {
    scan_msg->ranges.assign(
      ranges_size,
      std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon);
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height || *iter_z < min_height) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height, max_height);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }

  this->laser_scan_publisher_->publish(std::move(scan_msg));
}

}  // namespace laserscan_multi_merger

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<laserscan_multi_merger::LaserscanMerger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
