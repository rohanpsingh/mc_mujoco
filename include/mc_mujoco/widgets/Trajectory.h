#pragma once

#include "MujocoWidget.h"

namespace mc_mujoco
{

template<typename T>
struct Trajectory : public MujocoWidget
{
  Trajectory(Client & client, const ElementId & id) : MujocoWidget(client, id) {}

  void data(const T & point, const mc_rtc::gui::LineConfig & config)
  {
    points_.push_back(point);
    config_ = config;
  }

  void data(const std::vector<T> & points, const mc_rtc::gui::LineConfig & config)
  {
    points_ = points;
    config_ = config;
  }

  void draw3D() override
  {
    if(points_.size() < 2)
    {
      return;
    }
    for(size_t i = 0; i < points_.size() - 1; ++i)
    {
      mclient_.draw_line(points_[i], points_[i + 1], config_.color);
    }
    if constexpr(std::is_same_v<T, sva::PTransformd>)
    {
      if(points_.size() < 10) // For "small" trajectories, display all points
      {
        for(const auto & p : points_)
        {
          mclient_.draw_frame(p);
        }
      }
      else // Otherwise draw the start and end points
      {
        mclient_.draw_frame(points_[0]);
        mclient_.draw_frame(points_.back());
      }
    }
    else
    {
      mclient_.draw_box(points_[0], Eigen::Matrix3d::Identity(), Eigen::Vector3d::Constant(0.04), config_.color);
      mclient_.draw_sphere(points_.back(), 0.04, config_.color);
    }
  }

private:
  std::vector<T> points_;
  mc_rtc::gui::LineConfig config_;
};

} // namespace mc_mujoco
