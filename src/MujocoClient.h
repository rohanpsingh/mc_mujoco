#pragma once

#include "Client.h"

#include "glfw3.h"
#include "imgui.h"

namespace mc_mujoco
{

using Client = mc_rtc::imgui::Client;
using ElementId = mc_rtc::imgui::ElementId;

/** mc_rtc ControllerClient
 *
 * Uses ImGuizmo to draw in the 3D space of Mujoco
 */
struct MujocoClient : public mc_rtc::imgui::Client
{
  using mc_rtc::imgui::Client::Client;

  void draw2D(GLFWwindow * window);

  inline const std::array<float, 16> & view() const noexcept
  {
    return view_;
  }

  inline const std::array<float, 16> & projection() const noexcept
  {
    return projection_;
  }

  void draw_polygon(const std::vector<Eigen::Vector3d> & points, const mc_rtc::gui::Color & color, double thickness);

protected:
  void xytheta(const ElementId & id,
               const ElementId & requestId,
               bool ro,
               const Eigen::Vector3d & xytheta,
               double altitude) override;

  void polygon(const ElementId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & points,
               const mc_rtc::gui::LineConfig & config) override;

  inline void polygon(const ElementId & id,
                      const std::vector<std::vector<Eigen::Vector3d>> & points,
                      const mc_rtc::gui::Color & color) override
  {
    polygon(id, points, mc_rtc::gui::LineConfig(color));
  }

private:
  std::array<float, 16> view_;
  std::array<float, 16> projection_;
  float width_;
  float height_;
  ImDrawList * drawList_;
};

} // namespace mc_mujoco
