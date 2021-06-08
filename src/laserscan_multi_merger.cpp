#include "rclcpp/rclcpp.hpp"
#include <string.h>
#include <tf2_ros/transform_listener.h>
//#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.hpp>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp> 
// #include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>

using namespace std;
using namespace pcl;

class LaserscanMerger : public rclcpp::Node
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback();
    void laserscan_topic_parser();

private:

    laser_geometry::LaserProjection projector_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

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
}

void LaserscanMerger::laserscan_topic_parser()
{
    std::map<std::string, std::vector<std::string> > topics = this->get_topic_names_and_types(); //first:name, second:type

    istringstream iss(laserscan_topics);
    vector<string> tokens;
    copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
    vector<string> tmp_input_topics;

    // make sure given topics are actual topics
    // make sure topic types are LaserScan
    for(int i=0;i<tokens.size();++i)
    {
        for (auto const& topic : topics)
        {
            if( (tokens[i].compare(topic.first) == 0) && (topic.second[0].compare("sensor_msgs/msg/LaserScan") == 0) )
            {
                tmp_input_topics.push_back(topic.first);
            }
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
            RCLCPP_INFO(this->get_logger(),"Subscribing to topics\t%ld", scan_subscribers.size());
            for(int i=0; i<input_topics.size(); ++i)
            {
                // workaround for std::bind https://github.com/ros2/rclcpp/issues/583
                std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback = std::bind(&LaserscanMerger::scanCallback,
                    this, std::placeholders::_1, input_topics[i]);
                scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan> (
                    input_topics[i].c_str(), 1, callback);
                clouds_modified[i] = false;
                cout << input_topics[i] << " ";
            }
        }
        else
            RCLCPP_INFO(this->get_logger(),"Not subscribed to any topic.");
    }
}

LaserscanMerger::LaserscanMerger()
    : Node("laser_multi_merger")
{
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter<std::string>("destination_frame", "dest_frame");
    this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
    this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
    this->declare_parameter<std::string>("laserscan_topics", "");

    this->declare_parameter("angle_min", -2.36);
    this->declare_parameter("angle_max", 2.36);
    this->declare_parameter("angle_increment", 0.0058);
    this->declare_parameter("scan_time", 0.0333333);
    this->declare_parameter("range_min", 0.45);
    this->declare_parameter("range_max", 25.0);

    this->laserscan_topic_parser();

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2> (cloud_destination_topic.c_str(), 1);
    laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan> (scan_destination_topic.c_str(), 1);

}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic)
{
    sensor_msgs::msg::PointCloud tmpCloud1;
    sensor_msgs::msg::PointCloud2 tmpCloud2, tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
    tf_buffer_->canTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, rclcpp::Duration(1));
    projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud2, *tf_buffer_, laser_geometry::channel_option::Distance);
    
    //sensor_msgs::convertPointCloudToPointCloud2(tmpCloud1,tmpCloud2);
    pcl_ros::transformPointCloud(this->destination_frame.c_str(), tmpCloud2, tmpCloud3, *tf_buffer_);

    for(int i=0; i<input_topics.size(); ++i)
    {
        if(topic.compare(input_topics[i]) == 0)
        {
            pcl_conversions::toPCL(tmpCloud3, clouds[i]);
            clouds_modified[i] = true;
        }
    }	

    // Count how many scans we have
    int totalClouds = 0;
    for(int i=0; i<clouds_modified.size(); ++i)
        if(clouds_modified[i])
            ++totalClouds;

    // Go ahead only if all subscribed scans have arrived
    if(totalClouds == clouds_modified.size())
    {
        pcl::PCLPointCloud2 merged_cloud = clouds[0];
        clouds_modified[0] = false;

        for(int i=1; i<clouds_modified.size(); ++i)
        {
            pcl::concatenate(merged_cloud, clouds[i], merged_cloud);
            clouds_modified[i] = false;
        }
        
        auto msg = sensor_msgs::msg::PointCloud2();
        pcl_conversions::moveFromPCL(merged_cloud, msg);
        this->point_cloud_publisher_->publish(msg);

        Eigen::MatrixXf points;
        getPointCloudAsEigen(merged_cloud,points);

        pointcloud_to_laserscan(points, &merged_cloud);
    }
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
    sensor_msgs::msg::LaserScan::SharedPtr output;

    output->header = pcl_conversions::fromPCL(merged_cloud->header);
    output->header.frame_id = destination_frame.c_str();
    output->header.stamp = this->get_clock()->now();
    output->angle_min = this->angle_min;
    output->angle_max = this->angle_max;
    output->angle_increment = this->angle_increment;
    output->time_increment = this->time_increment;
    output->scan_time = this->scan_time;
    output->range_min = this->range_min;
    output->range_max = this->range_max;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for(int i=0; i<points.cols(); i++)
    {
        const float &x = points(0,i);
        const float &y = points(1,i);
        const float &z = points(2,i);

        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
            //RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y*y+x*x;
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_) {
            //RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            //RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;


        if (output->ranges[index] * output->ranges[index] > range_sq)
            output->ranges[index] = sqrt(range_sq);
    }

    this->laser_scan_publisher_->publish(*output);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserscanMerger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
