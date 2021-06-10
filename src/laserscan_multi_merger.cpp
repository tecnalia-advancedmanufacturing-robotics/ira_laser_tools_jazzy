#include "rclcpp/rclcpp.hpp"
#include <string.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
//#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.hpp>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
// #include "pcl_ros/point_cloud.h"

#include<iostream>
#include<algorithm>
#include <thread>
#include <chrono>

using namespace std;
using namespace pcl;

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
    void pointcloud_to_laserscan(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
    void reconfigureCallback();
    void laserscan_topic_parser();
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

private:

    laser_geometry::LaserProjection projector_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

    vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr > scan_subscribers;
    vector<bool> clouds_modified;
    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    double inf_epsilon;
    bool use_inf;

    double min_height_;
    double max_height_;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

void LaserscanMerger::reconfigureCallback()
{
    this->get_parameter("destination_frame", this->destination_frame);
    this->get_parameter("cloud_destination_topic", this->cloud_destination_topic);
    this->get_parameter("scan_destination_topic", this->scan_destination_topic);
    this->get_parameter("laserscan_topics", this->laserscan_topics);

    this->get_parameter("angle_min", this->angle_min);
    this->get_parameter("angle_max", this->angle_max);
    this->get_parameter("angle_increment", this->angle_increment);
    this->get_parameter("time_increment", this->time_increment);
    this->get_parameter("scan_time", this->scan_time);
    this->get_parameter("range_min", this->range_min);
    this->get_parameter("range_max", this->range_max);
    this->get_parameter("inf_epsilon", this->inf_epsilon);
    this->get_parameter("use_inf", this->use_inf); 
    this->get_parameter("min_height", this->min_height_);
    this->get_parameter("max_height", this->max_height_);
}

void LaserscanMerger::laserscan_topic_parser()
{
    // Necessity for sleep workaround to get topics
    // https://github.com/ros2/ros2/issues/1057
    std::map<std::string, std::vector<std::string> > topics;
    uint r = 0;
    while (topics.size() < 3){
        topics = this->get_topic_names_and_types(); //first:name, second:type
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (++r > 5){
            RCLCPP_ERROR(this->get_logger(),"No published topics found!");
            return;
        }
    }

    std::vector<std::string> published_scan_topics;
    for (auto const &topic: topics)
     {
         if (topic.second[0].compare("sensor_msgs/msg/LaserScan") == 0)
         {
            std::string topic_name = topic.first;
            if (topic_name.at(0) == '/')
                topic_name = topic_name.substr(1);
            published_scan_topics.push_back(topic_name);
         }
     }

    istringstream iss(this->laserscan_topics);
    vector<string> tokens ((istream_iterator<string>(iss)), istream_iterator<string>());
    vector<string> tmp_input_topics;

    // make sure given topics are published LaserScan topics
    for(auto const& token: tokens)
    {
        if (std::find(published_scan_topics.begin(), published_scan_topics.end(), token) != published_scan_topics.end())
        {
            tmp_input_topics.push_back(token);
        }
        else
        {
             RCLCPP_WARN(
                 this->get_logger(),
                 "Topic %s [sensor_msg/LaserScan] does not seem to be published yet. Could not subscribe.",
                 token);
        }
    }

    // clean up duplicate topics
    sort(tmp_input_topics.begin(),tmp_input_topics.end());
    std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
    tmp_input_topics.erase(last, tmp_input_topics.end());


    // Do not re-subscribe if the topics are the same
    if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
    {

        // Unsubscribe from previous topics
        // Not supported by ROS2 yet
        // for(int i=0; i<scan_subscribers.size(); ++i)
        //    scan_subscribers[i].shutdown();

        input_topics = tmp_input_topics;
        if(input_topics.size() > 0)
        {
            scan_subscribers.resize(input_topics.size());
            clouds_modified.resize(input_topics.size());
            clouds.resize(input_topics.size());
            std::stringstream output_info;
            std::copy(input_topics.begin(), input_topics.end(),std::ostream_iterator<std::string>(output_info," "));
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribing to %ld topics: %s.", scan_subscribers.size(), output_info.str().c_str());
            for(uint i=0; i<input_topics.size(); ++i)
            {
                // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
                std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback = std::bind(&LaserscanMerger::scanCallback,
                    this, std::placeholders::_1, input_topics[i]);
                scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan> (
                    input_topics[i].c_str(), 1, callback);
            RCLCPP_INFO(
                this->get_logger(),
                "Subscribed to %s.", input_topics[i].c_str());
                clouds_modified[i] = false;
            }
        }
        else
            RCLCPP_INFO(this->get_logger(),"Not subscribed to any topic.");
    }
}

LaserscanMerger::LaserscanMerger()
    : Node("laser_multi_merger")
{
    this->declare_parameter<std::string>("destination_frame", "dest_frame");
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    this->declare_parameter<std::string>("scan_destination_topic", "/merged_scan");
    this->declare_parameter<std::string>("laserscan_topics", "");

    this->declare_parameter("min_height", std::numeric_limits<double>::min());
    this->declare_parameter("max_height", std::numeric_limits<double>::max());

    this->declare_parameter("angle_min", -2.36);
    this->declare_parameter("angle_max", 2.36);
    this->declare_parameter("angle_increment", 0.0058);
    this->declare_parameter("scan_time", 0.0333333);
    this->declare_parameter("range_min", 0.45);
    this->declare_parameter("range_max", 25.0);
    this->declare_parameter("use_inf", true);
    this->declare_parameter("inf_epsilon", 1.0);
    this->reconfigureCallback();

    this->laserscan_topic_parser();


    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (this->cloud_destination_topic.c_str(), 1);
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan> (this->scan_destination_topic.c_str(), 1);

}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic)
{
    sensor_msgs::msg::PointCloud2 singleScanCloud, tmpCloud3;

    // transform scan if necessary
    // if (scan->header.frame_id != destination_frame)
    tf2::TimePoint time_point = tf2_ros::fromMsg(scan->header.stamp);
    //std::string *err_msg;
    bool canTransform = tf_buffer_->canTransform(destination_frame.c_str(), scan->header.frame_id.c_str(), time_point, tf2::durationFromSec(1.0));
    if (!canTransform){
        RCLCPP_WARN(
            this->get_logger(),
            "Could not look up transform from %s to %s at time [%ld.%ld]", 
            scan->header.frame_id.c_str(), destination_frame.c_str(), 
            scan->header.stamp.sec, scan->header.stamp.nanosec);
    }
    else
        RCLCPP_DEBUG(
            this->get_logger(),
            "Transform available"
        );
    //try{
        projector_.transformLaserScanToPointCloud(destination_frame.c_str(), *scan, singleScanCloud, *tf_buffer_);
        //pcl_ros::transformPointCloud(destination_frame.c_str(), tmpCloud2, tmpCloud3, *tf_buffer_);
    // }
    // catch(tf2::TransformException &ex){
    //     RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    // }

    for(uint i=0; i<input_topics.size(); ++i)
    {
        if(topic.compare(input_topics[i]) == 0)
        {   
            pcl_conversions::toPCL(singleScanCloud, clouds[i]);
            clouds_modified[i] = true;
        }
    }	

    // Count how many scans we have
    uint totalClouds = 0;
    for(uint i=0; i<clouds_modified.size(); ++i)
        if(clouds_modified[i])
            ++totalClouds;

    // Go ahead only if all subscribed scans have arrived
    if(totalClouds == clouds_modified.size())
    {
        pcl::PCLPointCloud2 merged_cloud = clouds[0];
        clouds_modified[0] = false;

        for(uint i=1; i<clouds_modified.size(); ++i)
        {
            pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
            clouds_modified[i] = false;
        }
        
        //auto msg = sensor_msgs::msg::PointCloud2();
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);
        this->point_cloud_publisher_->publish(*cloud_msg);

        pointcloud_to_laserscan(cloud_msg);
    }
}

void LaserscanMerger::pointcloud_to_laserscan(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
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
        scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
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

        if (*iter_z > max_height_ || *iter_z < min_height_) {
        RCLCPP_DEBUG(
            this->get_logger(),
            "rejected for height %f not in range (%f, %f)\n",
            *iter_z, min_height_, max_height_);
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserscanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
