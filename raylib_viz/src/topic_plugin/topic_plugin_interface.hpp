#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class TopicPluginInterface
{
protected:
  rclcpp::Node * node_;

public:
  TopicPluginInterface(rclcpp::Node * node) : node_(node) {}
  virtual ~TopicPluginInterface() {}
  virtual void visualize() = 0;
  virtual void init(){};
  virtual void init_drawing(){};
  virtual void preprocess(){};
};
