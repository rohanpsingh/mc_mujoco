#pragma once

#include "details/TransformBase.h"

namespace mc_mujoco
{

struct Rotation : public TransformBase<ControlAxis::ROTATION>
{
  Rotation(Client & client, const ElementId & id, const ElementId & reqId) : TransformBase(client, id, reqId) {}

  void draw3D() override
  {
    TransformBase::draw3D();
    mclient_.draw_frame(marker_.pose());
  }
};

} // namespace mc_mujoco
