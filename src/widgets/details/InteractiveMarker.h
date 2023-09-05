#pragma once

#include <memory>

#include <SpaceVecAlg/SpaceVecAlg>

#include "ControlAxis.h"

namespace mc_mujoco
{

struct InteractiveMarker
{
  InteractiveMarker(const sva::PTransformd & pose = sva::PTransformd::Identity(), ControlAxis mask = ControlAxis::NONE);

  void mask(ControlAxis mask);

  void pose(const sva::PTransformd & pose);

  inline bool active() const noexcept
  {
    return active_;
  }

  inline const sva::PTransformd & pose() const noexcept
  {
    return pose_;
  }

  bool draw(const std::array<float, 16> & view, const std::array<float, 16> & projection);

private:
  sva::PTransformd pose_;
  int operation_;
  bool active_ = false;
  int id_;
  static int next_id_;
};

using InteractiveMarkerPtr = std::unique_ptr<InteractiveMarker>;

} // namespace mc_mujoco
