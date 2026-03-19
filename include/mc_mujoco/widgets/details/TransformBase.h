#pragma once

#include "../MujocoWidget.h"
#include "InteractiveMarker.h"

namespace mc_mujoco
{

template<ControlAxis ctl>
struct TransformBase : public MujocoWidget
{
  TransformBase(Client & client, const ElementId & id, const ElementId & requestId)
  : MujocoWidget(client, id), requestId_(requestId), marker_(sva::PTransformd::Identity(), ControlAxis::NONE)
  {
  }

  ~TransformBase() override = default;

  void data(bool ro, const sva::PTransformd & pos)
  {
    marker_.mask(ro ? ControlAxis::NONE : ctl);
    marker_.pose(pos);
  }

  void draw3D() override
  {
    const auto & pos = marker_.pose();
    if(marker_.draw(mclient_.view(), mclient_.projection()))
    {
      if constexpr(ctl == ControlAxis::TRANSLATION)
      {
        client.send_request(requestId_, pos.translation());
      }
      else if constexpr(ctl == ControlAxis::ROTATION)
      {
        client.send_request(requestId_, pos.rotation());
      }
      else if constexpr(ctl == ControlAxis::ALL)
      {
        client.send_request(requestId_, pos);
      }
      else if constexpr(ctl == ControlAxis::XYTHETA || ctl == ControlAxis::XYZTHETA)
      {
        Eigen::VectorXd data = Eigen::VectorXd::Zero(4);
        const auto & t = pos.translation();
        auto yaw = mc_rbdyn::rpyFromMat(pos.rotation()).z();
        data(0) = t.x();
        data(1) = t.y();
        data(2) = yaw;
        data(3) = t.z();
        client.send_request(requestId_, data);
      }
    }
  }

protected:
  ElementId requestId_;
  InteractiveMarker marker_;
};

} // namespace mc_mujoco
