#include "viewer3d.hpp"

#include "topic_plugin/topic_plugin.hpp"

#include <iostream>
Viewer3D::Viewer3D(rclcpp::Node * node) : node_(node)
{
  camera_.position = Vector3{0.0f, 10.0f, 10.0f};  // Camera position
  camera_.target = Vector3{0.0f, 0.0f, 0.0f};      // Camera looking at point
  camera_.up = Vector3{0.0f, 1.0f, 0.0f};          // Camera up vector (rotation towards target)
  camera_.fovy = 45.0f;                            // Camera field-of-view Y
  camera_.projection = CAMERA_PERSPECTIVE;         // Camera mode type

  topic_plugins_.push_back(createPlugin<PointCloudPlugin>());
  topic_plugins_.push_back(createPlugin<ObjectsPlugin>());
}

Viewer3D::~Viewer3D()
{
}

void Viewer3D::setViewerFrame(std::string viewer_frame)
{
  viewer_frame_ = viewer_frame;
}

void Viewer3D::init_drawing()
{
  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->init_drawing();
  }
}

void Viewer3D::visualize()
{
  ClearBackground(RAYWHITE);

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->preprocess();
  }

  BeginMode3D(camera_);

  for (const auto & topic_plugin : topic_plugins_) {
    topic_plugin->visualize();
  }

  DrawGrid(10, 1.0f);

  EndMode3D();
  DrawFPS(10, 10);
}

template <typename T>
std::unique_ptr<T> Viewer3D::createPlugin()
{
  auto plugin = std::make_unique<T>(node_);
  plugin->init();
  return plugin;
}
