#pragma once
#include "raylib.h"
#include "topic_plugin/topic_plugin_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>
class Viewer3D
{
private:
  std::string viewer_frame_;
  Camera3D camera_;

  template <typename T>
  std::unique_ptr<T> createPlugin();
  std::vector<std::unique_ptr<TopicPluginInterface>> topic_plugins_;
  rclcpp::Node * node_;

public:
  Viewer3D(rclcpp::Node * node);
  ~Viewer3D();
  void setViewerFrame(std::string viewer_frame);
  void visualize();
  void init_drawing();
};
