#include "window.hpp"

#include "viewer3d.hpp"

Window::Window(rclcpp::Node * node) : node_(node)
{
  SetConfigFlags(FLAG_WINDOW_RESIZABLE);
  InitWindow(window_width_, window_height_, "raylib viz");
}

Window::~Window()
{
  CloseWindow();
}

void Window::run(std::function<bool()> continueLoop)
{
  Viewer3D viewer3d(node_);

  SetTargetFPS(fps_);  // Set our game to run at 60 frames-per-second
  ClearBackground(WHITE);
  viewer3d.init_drawing();

  while (continueLoop())  // Detect window close button or ESC key
  {
    rclcpp::spin_some(node_->get_node_base_interface());

    BeginDrawing();
    viewer3d.visualize();
    EndDrawing();
  }
}
