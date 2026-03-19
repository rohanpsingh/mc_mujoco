#include "InteractiveMarker.h"

#include "imgui.h"

#include "ImGuizmo.h"

#include <SpaceVecAlg/Conversions.h>

namespace mc_mujoco
{

static inline bool has(ControlAxis mask, ControlAxis value)
{
  return static_cast<std::underlying_type_t<ControlAxis>>(mask & value) != 0;
}

static inline ImGuizmo::OPERATION convert(ControlAxis mask)
{
  ImGuizmo::OPERATION out = static_cast<ImGuizmo::OPERATION>(0);
#define HANDLE(CA, OP) \
  if(has(mask, CA))    \
  {                    \
    out = out | OP;    \
  }
  HANDLE(ControlAxis::TX, ImGuizmo::OPERATION::TRANSLATE_X)
  HANDLE(ControlAxis::TY, ImGuizmo::OPERATION::TRANSLATE_Y)
  HANDLE(ControlAxis::TZ, ImGuizmo::OPERATION::TRANSLATE_Z)
  HANDLE(ControlAxis::RX, ImGuizmo::OPERATION::ROTATE_X)
  HANDLE(ControlAxis::RY, ImGuizmo::OPERATION::ROTATE_Y)
  HANDLE(ControlAxis::RZ, ImGuizmo::OPERATION::ROTATE_Z)
#undef HANDLE
  return out;
}

int InteractiveMarker::next_id_ = 0;

InteractiveMarker::InteractiveMarker(const sva::PTransformd & pose, ControlAxis mask) : pose_(pose), id_(next_id_++)
{
  this->mask(mask);
}

void InteractiveMarker::mask(ControlAxis mask)
{
  operation_ = convert(mask);
}

void InteractiveMarker::pose(const sva::PTransformd & pose)
{
  if(!active_)
  {
    pose_ = pose;
  }
}

bool InteractiveMarker::draw(const std::array<float, 16> & view, const std::array<float, 16> & projection)
{
  if(operation_ == 0)
  {
    return false;
  }
  ImGuizmo::SetID(id_);
  ImGuizmo::SetGizmoSizeWorldSpace(0.15f);
  Eigen::Matrix<float, 4, 4> mat = sva::conversions::toHomogeneous(pose_.cast<float>());
  auto op = static_cast<ImGuizmo::OPERATION>(operation_);
  bool changed = ImGuizmo::Manipulate(view.data(), projection.data(), op, ImGuizmo::MODE::LOCAL, mat.data());
  active_ = ImGuizmo::IsUsing();
  if(changed)
  {
    pose_ = sva::conversions::fromHomogeneous(mat).cast<double>();
  }
  return changed;
}

} // namespace mc_mujoco
