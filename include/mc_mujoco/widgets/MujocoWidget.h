#pragma once

#include "../MujocoClient.h"
#include "Widget.h"

namespace mc_mujoco
{

struct MujocoWidget : public mc_rtc::imgui::Widget
{
  MujocoWidget(Client & client, const ElementId & id)
  : mc_rtc::imgui::Widget(client, id), mclient_(static_cast<MujocoClient &>(client))
  {
  }

protected:
  MujocoClient & mclient_;
};

} // namespace mc_mujoco
