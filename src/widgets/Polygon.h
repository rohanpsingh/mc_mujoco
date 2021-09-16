#pragma once

#include "MujocoWidget.h"

namespace mc_mujoco
{

struct Polygon : public MujocoWidget
{
  Polygon(Client & client, const ElementId & id) : MujocoWidget(client, id) {}

  void data(const std::vector<std::vector<Eigen::Vector3d>> & points, const mc_rtc::gui::LineConfig & config)
  {
    if(points_ != points)
    {
      points_ = points;
    }
    config_ = config;
  }

  void draw3D() override
  {
    for(const auto & p : points_)
    {
      // FIXME Style is not supported by imgui drawing API
      mclient_.draw_polygon(p, config_.color, config_.width);
    }
  }

private:
  std::vector<std::vector<Eigen::Vector3d>> points_;
  mc_rtc::gui::LineConfig config_;
};

} // namespace mc_mujoco
