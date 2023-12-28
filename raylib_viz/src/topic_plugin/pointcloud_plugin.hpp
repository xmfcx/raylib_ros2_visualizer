#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iostream>

class PointCloudPlugin : public TopicPluginInterface
{
public:
  PointCloudPlugin(rclcpp::Node * node) : TopicPluginInterface(node) {}

  void init() override
  {
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensing/lidar/top/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&PointCloudPlugin::onPointCloud, this, std::placeholders::_1));
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::cout << "PointCloudPlugin::onPointCloud()" << std::endl;
    buffer_.addData(msg);
  }

  void visualize() override
  {
    const auto message = buffer_.getDataByTimestamp(
      std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
//      if (message->timestamp == prev_timestamp_) {
//        return;
//      }
      const auto & data = message->data;
      sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");
      //   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      //     //        DrawSphere(Vector3{*iter_x, *iter_y, *iter_z}, 0.05, RED);
      //     DrawPoint3D(Vector3{*iter_x, *iter_y, *iter_z}, RED);
      //   }
      rlBegin(RL_LINES);
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        rlVertex3f(-(*iter_y), *iter_z, -(*iter_x));
        rlVertex3f(-(*iter_y), *iter_z + 0.05f, -(*iter_x));
      }
      rlEnd();
      prev_timestamp_ = message->timestamp;
    }
  }

private:
  MessageBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::chrono::system_clock::time_point prev_timestamp_;
};
