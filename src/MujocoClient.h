#pragma once

#include "Client.h"

#include "glfw3.h"
#include "imgui.h"
#include "mujoco.h"

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

  void draw3D();

  inline const std::array<float, 16> & view() const noexcept
  {
    return view_;
  }

  inline const std::array<float, 16> & projection() const noexcept
  {
    return projection_;
  }

  inline const std::vector<mjvGeom> & geoms() const noexcept
  {
    return geoms_;
  }

  void draw_arrow(const Eigen::Vector3d & from,
                  const Eigen::Vector3d & to,
                  double shaft_diam,
                  double head_diam,
                  double head_len,
                  const mc_rtc::gui::Color & color) noexcept;

  void draw_frame(const sva::PTransformd & pos, double size = 0.1) noexcept;

  void draw_polygon(const std::vector<Eigen::Vector3d> & points,
                    const mc_rtc::gui::Color & color,
                    double thickness) noexcept;

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

  void force(const ElementId & id,
             const ElementId & requestId,
             const sva::ForceVecd & force,
             const sva::PTransformd & pos,
             const mc_rtc::gui::ForceConfig & forceConfig,
             bool ro) override;

  void arrow(const ElementId & id,
             const ElementId & requestId,
             const Eigen::Vector3d & start,
             const Eigen::Vector3d & end,
             const mc_rtc::gui::ArrowConfig & config,
             bool ro) override;

private:
  std::array<float, 16> view_;
  std::array<float, 16> projection_;
  std::vector<mjvGeom> geoms_;
  Eigen::Matrix4f mvp_;
  float width_;
  float height_;
  ImDrawList * drawList_;

  ImVec2 to_screen(const Eigen::Vector3d & point);
};

} // namespace mc_mujoco
