#pragma once
#include "raylib.h"

#include <rclcpp/rclcpp.hpp>

#include <functional>
class Window
{
private:
  int window_width_ = 800;
  int window_height_ = 450;
  int fps_ = 300;
  rclcpp::Node * node_;

public:
  explicit Window(rclcpp::Node * node);
  ~Window();
  void run(std::function<bool()> continueLoop);
};
