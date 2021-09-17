#pragma once

#include "details/TransformBase.h"

namespace mc_mujoco
{

struct Point3D : public TransformBase<ControlAxis::TRANSLATION>
{
  Point3D(Client & client, const ElementId & id, const ElementId & reqId) : TransformBase(client, id, reqId) {}

  ~Point3D() override = default;

  void data(bool ro, const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & config)
  {
    TransformBase::data(ro, pos);
    config_ = config;
  }

  void draw3D() override
  {
    TransformBase::draw3D();
    mclient_.draw_sphere(marker_.pose().translation(), config_.scale, config_.color);
  }

private:
  mc_rtc::gui::PointConfig config_;
};

} // namespace mc_mujoco
