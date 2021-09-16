#include "widgets/Polygon.h"
#include "widgets/XYTheta.h"

#include "imgui.h"

namespace mc_mujoco
{

namespace internal
{

/** Convert an mc_rtc Color to an imgui color */
inline ImU32 to_imu32(const mc_rtc::gui::Color & color) noexcept
{
  return ImGui::ColorConvertFloat4ToU32({static_cast<float>(color.r), static_cast<float>(color.g),
                                         static_cast<float>(color.b), static_cast<float>(color.a)});
}

/** Convert a Vector3d to an homogeneous Vector4f */
inline Eigen::Vector4f to_homo(const Eigen::Vector3d & p) noexcept
{
  return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), 1.0f};
}

/** Convert a world coordinate point into 2D screen space, maths taken from ImGuizmo.cpp */
inline ImVec2 to_screen(const Eigen::Vector4f & point,
                        const Eigen::Ref<Eigen::Matrix4f> & mvp,
                        float width,
                        float height) noexcept
{
  Eigen::Vector4f out = mvp * point;
  out *= 0.5f / out.w();
  out += Eigen::Vector4f{0.5f, 0.5f, 0.0f, 0.0f};
  out.y() = 1.0f - out.y();
  out.x() *= width;
  out.y() *= height;
  return ImVec2{out.x(), out.y()};
}

} // namespace internal

void MujocoClient::draw2D(GLFWwindow * window)
{
  glGetFloatv(GL_MODELVIEW_MATRIX, view_.data());
  glGetFloatv(GL_PROJECTION_MATRIX, projection_.data());

  int width;
  int height;
  glfwGetWindowSize(window, &width, &height);
  width_ = static_cast<float>(width);
  height_ = static_cast<float>(height);
  mc_rtc::imgui::Client::draw2D({width_, height_});

  ImGui::PushStyleColor(ImGuiCol_WindowBg, 0);
  ImGui::PushStyleColor(ImGuiCol_Border, 0);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);

  const ImU32 flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar
                      | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings
                      | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus;
  ImGui::Begin("gizmo", NULL, flags);
  drawList_ = ImGui::GetWindowDrawList();
  ImGui::End();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(2);
}

void MujocoClient::xytheta(const ElementId & id,
                           const ElementId & requestId,
                           bool ro,
                           const Eigen::Vector3d & xytheta,
                           double altitude)
{
  widget<XYTheta>(id, requestId).data(ro, xytheta, altitude);
}

void MujocoClient::polygon(const ElementId & id,
                           const std::vector<std::vector<Eigen::Vector3d>> & points,
                           const mc_rtc::gui::LineConfig & config)
{
  widget<Polygon>(id).data(points, config);
}

void MujocoClient::draw_polygon(const std::vector<Eigen::Vector3d> & points,
                                const mc_rtc::gui::Color & color,
                                double thickness)
{
  if(points.size() == 0)
  {
    return;
  }
  auto view = Eigen::Map<Eigen::Matrix4f>(view_.data());
  auto projection = Eigen::Map<Eigen::Matrix4f>(projection_.data());
  Eigen::Matrix4f mvp = projection * view;
  internal::to_screen(internal::to_homo(Eigen::Vector3d::Zero()), mvp, width_, height_);
  auto lineTo = [&](const Eigen::Vector3d & p) {
    drawList_->PathLineTo(internal::to_screen(internal::to_homo(p), mvp, width_, height_));
  };
  for(const auto & p : points)
  {
    lineTo(p);
  }
  lineTo(points[0]);
  drawList_->PathStroke(internal::to_imu32(color), 0, static_cast<float>(thickness));
}

} // namespace mc_mujoco
