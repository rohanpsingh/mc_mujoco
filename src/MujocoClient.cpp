#ifndef IMGUI_DEFINE_MATH_OPERATORS
#  define IMGUI_DEFINE_MATH_OPERATORS
#endif
#include "imgui.h"
#include "imgui_internal.h" // for ImVec2 operations

#include "widgets/Arrow.h"
#include "widgets/Force.h"
#include "widgets/Point3D.h"
#include "widgets/Polygon.h"
#include "widgets/Polyhedron.h"
#include "widgets/Rotation.h"
#include "widgets/Trajectory.h"
#include "widgets/Transform.h"
#include "widgets/Visual.h"
#include "widgets/XYTheta.h"

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

inline std::array<float, 4> to_rgba(const mc_rtc::gui::Color & color) noexcept
{
  return {static_cast<float>(color.r), static_cast<float>(color.g), static_cast<float>(color.b),
          static_cast<float>(color.a)};
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

inline ImVec2 to_screen(const Eigen::Vector3d & point,
                        const Eigen::Ref<Eigen::Matrix4f> & mvp,
                        float width,
                        float height) noexcept
{
  return to_screen(to_homo(point), mvp, width, height);
}

} // namespace internal

#ifdef USE_UI_ADAPTER
void MujocoClient::draw2D(mujoco::PlatformUIAdapter & window)
#else
void MujocoClient::draw2D(GLFWwindow * window)
#endif
{
  glGetFloatv(GL_MODELVIEW_MATRIX, view_.data());
  glGetFloatv(GL_PROJECTION_MATRIX, projection_.data());
  auto view = Eigen::Map<Eigen::Matrix4f>(view_.data());
  auto projection = Eigen::Map<Eigen::Matrix4f>(projection_.data());
  mvp_ = projection * view;

  int width;
  int height;
#ifdef USE_UI_ADAPTER
  std::tie(width, height) = window.GetWindowSize();
#else
  glfwGetWindowSize(window, &width, &height);
#endif
  width_ = static_cast<float>(width);
  height_ = static_cast<float>(height);

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

  mc_rtc::imgui::Client::draw2D({width_, height_});
}

void MujocoClient::draw3D()
{
  geoms_.clear();
  mc_rtc::imgui::Client::draw3D();
}

void MujocoClient::updateScene(mjvScene & scene)
{
  for(const auto & g : geoms_)
  {
    if(scene.ngeom < scene.maxgeom)
    {
      scene.geoms[scene.ngeom] = g;
      scene.ngeom++;
    }
    else
    {
      static bool warned_once = false;
      if(!warned_once)
      {
        mc_rtc::log::critical(
            "Too many geometric objects in the scene, increase maxgeom in model, some elements will not be visible");
        warned_once = true;
      }
      break;
    }
  }
}

void MujocoClient::point3d(const ElementId & id,
                           const ElementId & requestId,
                           bool ro,
                           const Eigen::Vector3d & pos,
                           const mc_rtc::gui::PointConfig & config)
{
  widget<Point3D>(id, requestId).data(ro, pos, config);
}

void MujocoClient::rotation(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos)
{
  widget<Rotation>(id, requestId).data(ro, pos);
}

void MujocoClient::transform(const ElementId & id, const ElementId & requestId, bool ro, const sva::PTransformd & pos)
{
  widget<TransformWidget>(id, requestId).data(ro, pos);
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

void MujocoClient::polyhedron(const ElementId & id,
                              const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                              const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                              const mc_rtc::gui::PolyhedronConfig & config)
{
  widget<Polyhedron>(id).data(triangles, colors, config);
}

void MujocoClient::arrow(const ElementId & id,
                         const ElementId & requestId,
                         const Eigen::Vector3d & start,
                         const Eigen::Vector3d & end,
                         const mc_rtc::gui::ArrowConfig & config,
                         bool ro)
{
  widget<Arrow>(id, requestId).data(start, end, config, ro);
}

void MujocoClient::force(const ElementId & id,
                         const ElementId & requestId,
                         const sva::ForceVecd & force,
                         const sva::PTransformd & pos,
                         const mc_rtc::gui::ForceConfig & config,
                         bool ro)
{
  widget<Force>(id, requestId).data(force, pos, config, ro);
}

void MujocoClient::trajectory(const ElementId & id,
                              const std::vector<Eigen::Vector3d> & points,
                              const mc_rtc::gui::LineConfig & config)
{
  widget<Trajectory<Eigen::Vector3d>>(id).data(points, config);
}

void MujocoClient::trajectory(const ElementId & id,
                              const std::vector<sva::PTransformd> & points,
                              const mc_rtc::gui::LineConfig & config)
{
  widget<Trajectory<sva::PTransformd>>(id).data(points, config);
}

void MujocoClient::trajectory(const ElementId & id,
                              const Eigen::Vector3d & point,
                              const mc_rtc::gui::LineConfig & config)
{
  widget<Trajectory<Eigen::Vector3d>>(id).data(point, config);
}

void MujocoClient::trajectory(const ElementId & id,
                              const sva::PTransformd & point,
                              const mc_rtc::gui::LineConfig & config)
{
  widget<Trajectory<sva::PTransformd>>(id).data(point, config);
}

void MujocoClient::visual(const ElementId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pos)
{
  widget<Visual>(id).data(visual, pos);
}

ImVec2 MujocoClient::to_screen(const Eigen::Vector3d & point)
{
  return internal::to_screen(point, mvp_, width_, height_);
}

void MujocoClient::draw_line(const Eigen::Vector3d & from,
                             const Eigen::Vector3d & to,
                             const mc_rtc::gui::Color & color_)
{
  geoms_.push_back({});
  auto & geom = geoms_.back();
  double length = (to - from).norm();
  double size[3] = {1.0, 1.0, length};
  Eigen::Matrix3d orientation =
      Eigen::Quaterniond::FromTwoVectors(to - from, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto color = internal::to_rgba(color_);
  mjv_initGeom(&geom, mjGEOM_LINE, size, from.data(), orientation.data(), color.data());
}

void MujocoClient::draw_box(const Eigen::Vector3d & center,
                            const Eigen::Matrix3d & orientation,
                            const Eigen::Vector3d & size,
                            const mc_rtc::gui::Color & color_)
{
  geoms_.push_back({});
  auto & geom = geoms_.back();
  auto color = internal::to_rgba(color_);
  mjv_initGeom(&geom, mjGEOM_BOX, size.data(), center.data(), orientation.data(), color.data());
}

void MujocoClient::draw_sphere(const Eigen::Vector3d & center, double radius, const mc_rtc::gui::Color & color_)
{
  geoms_.push_back({});
  auto & geom = geoms_.back();
  double size[3] = {radius, radius, radius};
  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  auto color = internal::to_rgba(color_);
  mjv_initGeom(&geom, mjGEOM_SPHERE, size, center.data(), mat.data(), color.data());
}

void MujocoClient::draw_arrow(const Eigen::Vector3d & from_,
                              const Eigen::Vector3d & to_,
                              double shaft_diam_,
                              double head_diam_,
                              double head_len_,
                              const mc_rtc::gui::Color & color_) noexcept
{
  geoms_.push_back({});
  auto & geom = geoms_.back();
  auto type = mjGEOM_ARROW;
  if(head_diam_ == 0.0 || head_len_ == 0.0)
  {
    type = mjGEOM_CYLINDER;
  }
  if(head_diam_ == shaft_diam_)
  {
    type = mjGEOM_ARROW1;
  }
  Eigen::Vector3d dir_ = (to_ - from_);
  // FIXME For some reason it seems size[2] is half of the arrow length?
  double size[3] = {shaft_diam_, shaft_diam_, 2 * dir_.norm()};
  Eigen::Matrix3d orientation = Eigen::Quaterniond::FromTwoVectors(dir_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto color = internal::to_rgba(color_);
  mjv_initGeom(&geom, type, size, from_.data(), orientation.data(), color.data());
}

void MujocoClient::draw_frame(const sva::PTransformd & pos, double size) noexcept
{
  auto draw_axis = [&](const Eigen::Vector3d & unit, const mc_rtc::gui::Color & color)
  { draw_arrow(pos.translation(), (sva::PTransformd{unit} * pos).translation(), 0.015, 0.015, 0.2 * size, color); };
  draw_axis(size * Eigen::Vector3d::UnitX(), mc_rtc::gui::Color::Red);
  draw_axis(size * Eigen::Vector3d::UnitY(), mc_rtc::gui::Color::Green);
  draw_axis(size * Eigen::Vector3d::UnitZ(), mc_rtc::gui::Color::Blue);
}

void MujocoClient::draw_polygon(const std::vector<Eigen::Vector3d> & points,
                                const mc_rtc::gui::Color & color_,
                                double thickness) noexcept
{
  if(points.size() == 0)
  {
    return;
  }
  auto lineTo = [&](const Eigen::Vector3d & p) { drawList_->PathLineTo(to_screen(p)); };
  for(const auto & p : points)
    for(size_t i = 0; i < points.size(); ++i)
    {
      const auto & p0 = points[i];
      const auto & p1 = points[(i + 1) % points.size()];
      draw_line(p0, p1, color_);
    }
}

void MujocoClient::draw_triangle(const std::array<Eigen::Vector3d, 3> & triangle_,
                                 const mc_rtc::gui::Color & color_) noexcept
{
  geoms_.push_back({});
  auto & geom = geoms_.back();
  Eigen::Vector3d e1 = triangle_[1] - triangle_[0];
  Eigen::Vector3d e2 = triangle_[2] - triangle_[0];
  Eigen::Vector3d normal = e1.cross(e2);

  Eigen::Vector3d lengths = {e1.norm(), e2.norm(), normal.norm()};
  e1.normalize();
  e2.normalize();
  normal.normalize();

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> xmat;
  xmat << e1[0], e2[0], normal[0], e1[1], e2[1], normal[1], e1[2], e2[2], normal[2];

  auto color = internal::to_rgba(color_);
  mjv_initGeom(&geom, mjGEOM_TRIANGLE, lengths.data(), triangle_[0].data(), xmat.data(), color.data());
}

void MujocoClient::draw_polyhedron(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles_,
                                   mc_rtc::gui::Color & color_) noexcept
{
  for(int i = 0; i < triangles_.size(); i++)
  {
    draw_triangle(triangles_[i], color_);
  }
}

} // namespace mc_mujoco
