#include <chrono>
#include <memory>
#include <iostream>
#include <vector>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class LaserscanToPointcloud : public rclcpp::Node {
  public:
    LaserscanToPointcloud()
    : Node("laserscan_to_pointcloud")
    {
      auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      
      // LiDAR Subscriber
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile, std::bind(&LaserscanToPointcloud::laser_callback, this, std::placeholders::_1));
      
      // Point Cloud Publisher
      cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

  private :
  
    void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {  
      std::vector<std::pair<double, double>> scan_map;
      std::string hd = msg->header.frame_id;
      double angle = msg->angle_min;
      for (int i=0; i<(int) msg->ranges.size(); i++) {
        if (!isinf(msg->ranges[i])) {
          double px = cos(angle) * msg->ranges[i];
          double py = sin(angle) * msg->ranges[i];
          scan_map.push_back(std::pair<double, double>(px, py));
        }
        angle += msg->angle_increment;
      }
      
      pcl::PointCloud<pcl::PointXYZRGB> cloud_;
      
      for (int i=0; i<(int) scan_map.size(); i++) {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(255, 255, 255);
        pt.x = scan_map[i].first;
        pt.y = scan_map[i].second;
        pt.z = 0.0;
        cloud_.points.push_back(pt);
      }
      
      pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud_, *pc2_msg_);
      pc2_msg_->header.frame_id = hd;
      pc2_msg_->header.stamp = now();
      cloud_pub_->publish(*pc2_msg_);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanToPointcloud>());
  rclcpp::shutdown();
  return 0;
}
