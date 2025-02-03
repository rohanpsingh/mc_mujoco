#pragma once

#include "MujocoWidget.h"

namespace mc_mujoco
{

struct Polyhedron : public MujocoWidget
{
  Polyhedron(Client & client, const ElementId & id) : MujocoWidget(client, id) {}

  void data(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
            const std::vector<std::array<mc_rtc::gui::Color, 3>> & color,
            const mc_rtc::gui::PolyhedronConfig & config)
  {
    if(triangles_ != triangles)
    {
      triangles_ = triangles;
    }

    config_ = config;
  }

  void draw3D() override
  {
    if(!show_)
    {
      return;
    }

    mclient_.draw_polyhedron(triangles_, config_.triangle_color);
  }

  void draw2D() override
  {
    ImGui::Checkbox(label(fmt::format("Show {}", id.name)).c_str(), &show_);
  }

private:
  std::vector<std::array<Eigen::Vector3d, 3>> triangles_;
  mc_rtc::gui::PolyhedronConfig config_;
  bool show_ = true;
};

} // namespace mc_mujoco
