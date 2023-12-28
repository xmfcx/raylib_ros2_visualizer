#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "topic_plugin_interface.hpp"
#include "util.hpp"
#include <raymath.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#define RLIGHTS_IMPLEMENTATION
#include "rlights.hpp"

#include <iostream>

class PointCloudPlugin : public TopicPluginInterface {
public:
  PointCloudPlugin(rclcpp::Node *node) : TopicPluginInterface(node) {}

  void init() override {
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&PointCloudPlugin::onPointCloud, this,
                  std::placeholders::_1));
    shader_path_lighting_ =
        node_->declare_parameter<std::string>("shader_path_lighting");
    shader_path_lighting_instancing_ =
        node_->declare_parameter<std::string>("shader_path_lighting_instancing");
  }

  void init_drawing() override {

    shader_ = std::make_unique<Shader>();
    *shader_ = LoadShader(
        TextFormat("/home/mfc/programs/raylib/build/examples/resources/"
                   "shaders/glsl330/lighting_instancing.vs"),
        TextFormat("/home/mfc/programs/raylib/build/examples/resources/"
                   "shaders/glsl330/lighting.fs"));

    //      // Get shader locations
    shader_->locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(*shader_, "mvp");
    shader_->locs[SHADER_LOC_VECTOR_VIEW] =
        GetShaderLocation(*shader_, "viewPos");
    shader_->locs[SHADER_LOC_MATRIX_MODEL] =
        GetShaderLocationAttrib(*shader_, "instanceTransform");
    // Create one light
    Vector3 lightDirection = {50.0f, 50.0f, 0.0f};
    stupid_rlights::CreateLight(stupid_rlights::LIGHT_DIRECTIONAL,
                                lightDirection, Vector3Zero(), WHITE, *shader_);

    // Set shader value: ambient light level
    int ambientLoc = GetShaderLocation(*shader_, "ambient");
    float ambientValues[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    SetShaderValue(*shader_, ambientLoc, ambientValues, SHADER_UNIFORM_VEC4);

    //      SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW],
    //                     Vector3{0.0f, 10.0f, 10.0f}, SHADER_UNIFORM_VEC3);

    // Initialization of the material
    material_ = std::make_unique<Material>();
    *material_ = LoadMaterialDefault();
    material_->shader = *shader_;
    material_->maps[MATERIAL_MAP_DIFFUSE].color = PURPLE;

    pointMesh_ = std::make_unique<Mesh>();
    const float cube_size = 0.05f;
    *pointMesh_ = GenMeshCube(cube_size, cube_size, cube_size);

    transformations_ = (Matrix *)RL_CALLOC(points_count_max_, sizeof(Matrix));
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::cout << "PointCloudPlugin::onPointCloud()" << std::endl;
    buffer_.addData(msg);
  }

  void visualize() override {
    const auto message = buffer_.getDataByTimestamp(
        std::chrono::system_clock::time_point(std::chrono::seconds(0)), false);
    if (message) {
      //      if (message->timestamp == prev_timestamp_) {
      //        return;
      //      }

      const auto &data = message->data;
      sensor_msgs::PointCloud2Iterator<float> iter_x(*data, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*data, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*data, "z");
      int points_count = message->data->width * message->data->height;
      if (points_count > points_count_max_) {
        points_count = points_count_max_;
      }
      // log points_count
      std::cout << "points_count: " << points_count << std::endl;

      for (int i = 0; i < points_count; i++) {
        const float x = -(*iter_y);
        const float y = *iter_z;
        const float z = -(*iter_x);
        transformations_[i] = MatrixTranslate(x, y, z);
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }

      DrawMeshInstanced(*pointMesh_, *material_, transformations_,
                        points_count);

      prev_timestamp_ = message->timestamp;
    }
  }

private:
  MessageBuffer<sensor_msgs::msg::PointCloud2::SharedPtr> buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::chrono::system_clock::time_point prev_timestamp_;
  std::unique_ptr<Shader> shader_;
  std::unique_ptr<Mesh> pointMesh_;
  std::unique_ptr<Material> material_;
  Matrix *transformations_;
  int points_count_max_ = 1e9;
  std::string shader_path_lighting_;
  std::string shader_path_lighting_instancing_;
};
