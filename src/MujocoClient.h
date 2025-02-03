#pragma once

#include "Client.h"

#include "glfw3.h"
#include "imgui.h"
#include "mujoco.h"

#ifdef USE_UI_ADAPTER
#  include "platform_ui_adapter.h"
#endif

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

#ifdef USE_UI_ADAPTER
  void draw2D(mujoco::PlatformUIAdapter & window);
#else
  void draw2D(GLFWwindow * window);
#endif

  void draw3D();

  void updateScene(mjvScene & scene);

  inline const std::array<float, 16> & view() const noexcept
  {
    return view_;
  }

  inline const std::array<float, 16> & projection() const noexcept
  {
    return projection_;
  }

  void draw_line(const Eigen::Vector3d & from, const Eigen::Vector3d & to, const mc_rtc::gui::Color & color);

  inline void draw_line(const sva::PTransformd & from, const sva::PTransformd & to, const mc_rtc::gui::Color & color)
  {
    draw_line(from.translation(), to.translation(), color);
  }

  void draw_box(const Eigen::Vector3d & center,
                const Eigen::Matrix3d & orientation,
                const Eigen::Vector3d & size,
                const mc_rtc::gui::Color & color);

  void draw_sphere(const Eigen::Vector3d & center, double radius, const mc_rtc::gui::Color & color);

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

  void draw_triangle(const std::array<Eigen::Vector3d, 3> & triangle_, const mc_rtc::gui::Color & color_) noexcept;

  void draw_polyhedron(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles_,
                       mc_rtc::gui::Color & color_) noexcept;

protected:
  void point3d(const ElementId & id,
               const ElementId & requestId,
               bool ro,
               const Eigen::Vector3d & pos,
               const mc_rtc::gui::PointConfig & config) override;

  void rotation(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;

  void transform(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos) override;

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

  void polyhedron(const ElementId & id,
                  const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                  const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                  const mc_rtc::gui::PolyhedronConfig & config) override;

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

  void trajectory(const ElementId & id,
                  const std::vector<Eigen::Vector3d> & points,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id,
                  const std::vector<sva::PTransformd> & points,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const ElementId & id,
                  const sva::PTransformd & point,
                  const mc_rtc::gui::LineConfig & config) override;

  void visual(const ElementId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pos) override;

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
