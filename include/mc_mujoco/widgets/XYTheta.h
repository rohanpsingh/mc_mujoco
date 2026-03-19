#pragma once

#include "details/TransformBase.h"

namespace mc_mujoco
{

struct XYTheta : public TransformBase<ControlAxis::XYZTHETA>
{
  XYTheta(Client & client, const ElementId & id, const ElementId & reqId) : TransformBase(client, id, reqId) {}

  void data(bool ro, const Eigen::Vector3d & xytheta, double altitude)
  {
    TransformBase::data(ro, {sva::RotZ(xytheta.z()), {xytheta.x(), xytheta.y(), altitude}});
  }

  void draw3D() override
  {
    TransformBase::draw3D();
  }
};

} // namespace mc_mujoco
