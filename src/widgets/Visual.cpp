#include "Visual.h"

namespace mc_mujoco
{

namespace internal
{

mc_rtc::gui::Color color(const rbd::parsers::Material & m)
{
  if(m.type == rbd::parsers::Material::Type::COLOR)
  {
    const auto & c = boost::get<rbd::parsers::Material::Color>(m.data);
    return {c.r, c.g, c.b, c.a};
  }
  return {1.0f, 1.0f, 1.0f, 1.0f};
}

} // namespace internal

Visual::Visual(Client & client, const ElementId & id) : MujocoWidget(client, id) {}

void Visual::data(const rbd::parsers::Visual & visual, const sva::PTransformd & pos)
{
  visual_ = visual;
  // Bake the visual origin in the position
  pos_ = visual.origin * pos;
}

void Visual::draw2D()
{
  ImGui::Checkbox(label(fmt::format("Show {}", id.name)).c_str(), &show_);
}

void Visual::draw3D()
{
  if(!show_)
  {
    return;
  }
  using Geometry = rbd::parsers::Geometry;
  using Type = rbd::parsers::Geometry::Type;
  auto handleMesh = [&]()
  {
    static bool warned_about_mesh = false;
    if(!warned_about_mesh)
    {
      mc_rtc::log::warning("[mc_mujoco] Does not support mesh Visual elements");
      warned_about_mesh = true;
    }
  };
  auto handleBox = [&]()
  {
    const auto & box = boost::get<Geometry::Box>(visual_.geometry.data);
    mclient_.draw_box(pos_.translation(), pos_.rotation(), box.size, internal::color(visual_.material));
  };
  auto handleCylinder = [&]()
  {
    const auto & cyl = boost::get<Geometry::Cylinder>(visual_.geometry.data);
    const auto & start = sva::PTransformd{Eigen::Vector3d{0, 0, -cyl.length / 2}} * pos_;
    const auto & end = sva::PTransformd{Eigen::Vector3d{0, 0, cyl.length / 2}} * pos_;
    mclient_.draw_arrow(start.translation(), end.translation(), 2 * cyl.radius, 0.0, 0.0,
                        internal::color(visual_.material));
  };
  auto handleSphere = [&]()
  {
    const auto & sphere = boost::get<rbd::parsers::Geometry::Sphere>(visual_.geometry.data);
    mclient_.draw_sphere(pos_.translation(), sphere.radius, internal::color(visual_.material));
  };
  switch(visual_.geometry.type)
  {
    case Type::MESH:
      handleMesh();
      break;
    case Type::BOX:
      handleBox();
      break;
    case Type::CYLINDER:
      handleCylinder();
      break;
    case Type::SPHERE:
      handleSphere();
      break;
    default:
      break;
  }
}

} // namespace mc_mujoco
