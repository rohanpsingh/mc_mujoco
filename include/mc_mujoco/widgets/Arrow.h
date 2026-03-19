#pragma once

#include "MujocoWidget.h"

#include "details/InteractiveMarker.h"

namespace mc_mujoco
{

struct Arrow : public MujocoWidget
{
  Arrow(Client & client, const ElementId & id, const ElementId & reqId) : MujocoWidget(client, id), requestId_(reqId) {}

  void data(const Eigen::Vector3d & start,
            const Eigen::Vector3d & end,
            const mc_rtc::gui::ArrowConfig & config,
            bool ro)
  {
    startMarker_.mask(ro ? ControlAxis::NONE : ControlAxis::TRANSLATION);
    startMarker_.pose(start);
    endMarker_.mask(ro ? ControlAxis::NONE : ControlAxis::TRANSLATION);
    endMarker_.pose(end);
    config_ = config;
  }

  void draw3D() override
  {
    if(!show_) return;

    const auto & start = startMarker_.pose().translation();
    const auto & end = endMarker_.pose().translation();
    mclient_.draw_arrow(start, end, config_.shaft_diam, config_.head_diam, config_.head_len, config_.color);
    bool changed = startMarker_.draw(mclient_.view(), mclient_.projection());
    if(endMarker_.draw(mclient_.view(), mclient_.projection()) || changed)
    {
      Eigen::Vector6d data;
      data << start, end;
      client.send_request(requestId_, data);
    }
  }

  void draw2D() override
  {
    ImGui::Checkbox(label(fmt::format("Show {}", id.name)).c_str(), &show_);
  }

private:
  ElementId requestId_;
  mc_rtc::gui::ArrowConfig config_;
  InteractiveMarker startMarker_;
  InteractiveMarker endMarker_;
  bool show_ = true;
};

} // namespace mc_mujoco
