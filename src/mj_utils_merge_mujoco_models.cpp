#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "pugixml/pugixml.hpp"

#include <mc_rtc/logging.h>

#include "mj_sim_impl.h"

namespace mc_mujoco
{

static pugi::xml_node get_child_or_create(pugi::xml_node & out, const char * name)
{
  auto child = out.child(name);
  if(!child)
  {
    child = out.append_child(name);
  }
  return child;
}

static void merge_mujoco_size(const pugi::xml_node & in, pugi::xml_node & out)
{
  static const char * attributes[] = {"njmax",        "nconmax",        "nstack",      "nuserdata",  "nkey",
                                      "nuser_body",   "nuser_jnt",      "nuser_geom",  "nuser_site", "nuser_cam",
                                      "nuser_tendon", "nuser_actuator", "nuser_sensor"};
  for(const auto & attr : attributes)
  {
    auto in_attr = in.attribute(attr);
    if(!in_attr)
    {
      continue;
    }
    auto out_attr = out.attribute(attr);
    if(out_attr)
    {
      out_attr.set_value(out_attr.as_int() + in_attr.as_int());
    }
    else
    {
      out.append_attribute(attr).set_value(in_attr.value());
    }
  }
}

static void merge_mujoco_node(const std::string & node,
                              const std::string & fileIn,
                              const pugi::xml_node & in,
                              pugi::xml_node & out,
                              const std::vector<std::string> & exclude = {})
{
  for(const auto & attr : in.attributes())
  {
    if(std::find(exclude.begin(), exclude.end(), attr.name()) != exclude.end())
    {
      continue;
    }
    auto out_attr = out.attribute(attr.name());
    if(!out_attr)
    {
      out.append_attribute(attr.name()).set_value(attr.value());
      continue;
    }
    if(strcmp(out_attr.value(), attr.value()) != 0)
    {
      mc_rtc::log::critical("[mc_mujoco] Different mujoco attributes when merging models, the first loaded value will "
                            "prevail (in {} node, attribute {}, value in {}: {}, value in merged model: {})",
                            node, attr.name(), fileIn, attr.value(), out_attr.value());
    }
  }
}

static void merge_mujoco_compiler(const std::string & fileIn, const pugi::xml_node & in, pugi::xml_node & out)
{
  merge_mujoco_node("compiler", fileIn, in, out, {"meshdir", "texturedir"});
}

static void merge_mujoco_option(const std::string & fileIn, const pugi::xml_node & in, pugi::xml_node & out)
{
  merge_mujoco_node("option", fileIn, in, out);
  const auto & flag = in.child("flag");
  if(flag)
  {
    auto flag_out = get_child_or_create(out, "flag");
    merge_mujoco_node("option/flag", fileIn, flag, flag_out);
  }
}

static void add_prefix(const std::string & prefix, pugi::xml_node & n, const char * attr, bool force = false)
{
  auto n_attr = [&]() {
    auto out = n.attribute(attr);
    if(!out && force)
    {
      out = n.append_attribute(attr);
    }
    return out;
  }();
  if(n_attr)
  {
    n_attr.set_value(fmt::format("{}_{}", prefix, n_attr.value()).c_str());
  }
}

static void add_prefix_recursively(const std::string & prefix,
                                   pugi::xml_node & out,
                                   const std::vector<std::string> & attrs)
{
  // In the composite tag we always want to add a prefix
  if(strcmp("composite", out.name()) == 0)
  {
    add_prefix(prefix, out, "prefix", true);
  }
  for(const auto & attr : attrs)
  {
    add_prefix(prefix, out, attr.c_str(), strcmp("freejoint", out.name()) == 0 && attr == "name");
  }
  for(auto & c : out.children())
  {
    add_prefix_recursively(prefix, c, attrs);
  }
}

static void merge_mujoco_default(const std::string & fileIn,
                                 const pugi::xml_node & in,
                                 pugi::xml_node & out,
                                 const std::string & robot)
{
  for(const auto & c : in.children())
  {
    if(strcmp(c.name(), "default") == 0)
    {
      auto c_out = out.append_copy(c);
      add_prefix_recursively(robot, c_out, {"class", "material", "hfield", "mesh", "target"});
    }
    else
    {
      auto c_out = get_child_or_create(out, c.name());
      merge_mujoco_node(fmt::format("default/{}", c.name()), fileIn, c, c_out);
    }
  }
}

static void merge_mujoco_visual(const std::string & fileIn, const pugi::xml_node & in, pugi::xml_node & out)
{
  for(const auto & c : in.children())
  {
    auto c_out = get_child_or_create(out, c.name());
    merge_mujoco_node(fmt::format("visual/{}", c.name()), fileIn, c, c_out);
  }
}

static void copy_and_add_prefix(const pugi::xml_node & in,
                                pugi::xml_node & out,
                                const char * name,
                                const std::string & prefix,
                                const std::vector<std::string> & attrs)
{
  for(const auto & c : in.children(name))
  {
    auto c_out = out.append_copy(c);
    for(const auto & attr : attrs)
    {
      add_prefix(prefix, c_out, attr.c_str());
    }
  }
}

static void merge_mujoco_asset(const pugi::xml_node & in,
                               pugi::xml_node & out,
                               const bfs::path & meshPath,
                               const bfs::path & texturePath,
                               const std::string & robot)
{
  auto update_name = [&](pugi::xml_node & n) { add_prefix(robot, n, "name"); };
  auto update_file = [&](pugi::xml_node & n, const bfs::path & dir) {
    auto n_file = n.attribute("file");
    if(n_file)
    {
      bfs::path n_path(n_file.value());
      if(!n_path.is_absolute())
      {
        n_file.set_value(bfs::absolute(dir / n_path).c_str());
      }
    }
  };
  for(const auto & hf : in.children("hfield"))
  {
    auto hf_out = out.append_copy(hf);
    update_name(hf_out);
  }
  auto copy_assets = [&](const char * type, const bfs::path & dir) {
    for(const auto & n : in.children(type))
    {
      auto n_out = out.append_copy(n);
      update_name(n_out);
      update_file(n_out, dir);
    }
  };
  for(const auto & s : in.children("skin"))
  {
    auto s_out = out.append_copy(s);
    update_name(s_out);
    update_file(s_out, meshPath);
    for(auto & bone : s_out.children("bone"))
    {
      add_prefix(robot, bone, "body");
    }
  }
  for(const auto & mat : in.children("material"))
  {
    auto mat_out = out.append_copy(mat);
    update_name(mat_out);
    add_prefix(robot, mat_out, "texture");
    add_prefix(robot, mat_out, "class");
  }
  copy_assets("texture", texturePath);
  copy_assets("mesh", meshPath);
}

static void merge_mujoco_contact(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  copy_and_add_prefix(in, out, "pair", robot, {"name", "class", "geom1", "geom2"});
  copy_and_add_prefix(in, out, "exclude", robot, {"name", "body1", "body2"});
}

static void merge_mujoco_equality(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  copy_and_add_prefix(in, out, "connect", robot, {"name", "class", "body1", "body2"});
  copy_and_add_prefix(in, out, "weld", robot, {"name", "class", "body1", "body2"});
  copy_and_add_prefix(in, out, "joint", robot, {"name", "class", "joint1", "joint2"});
  copy_and_add_prefix(in, out, "tendon", robot, {"name", "class", "tendon1", "tendon2"});
  copy_and_add_prefix(in, out, "distance", robot, {"name", "class", "geom1", "geom2"});
}

static void merge_mujoco_tendon(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  copy_and_add_prefix(in, out, "spatial", robot, {"name", "class", "material"});
  for(auto & spatial : out.children("spatial"))
  {
    for(auto & site : spatial.children("site"))
    {
      add_prefix(robot, site, "site");
    }
    for(auto & site : spatial.children("geom"))
    {
      add_prefix(robot, site, "geom");
      add_prefix(robot, site, "sidesite");
    }
  }
  copy_and_add_prefix(in, out, "fixed", robot, {"name", "class", "material"});
  for(auto & fixed : out.children("fixed"))
  {
    for(auto & joint : fixed.children("joint"))
    {
      add_prefix(robot, joint, "joint");
    }
  }
}

static void merge_mujoco_actuator(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  for(const auto & c : in.children())
  {
    auto c_out = out.append_copy(c);
    for(const auto & attr : {"name", "class", "joint", "jointinparent", "site", "tendon", "cranksite", "slidersite"})
    {
      add_prefix(robot, c_out, attr);
    }
  }
}

static void merge_mujoco_sensor(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  for(const auto & c : in.children())
  {
    auto c_out = out.append_copy(c);
    for(const auto & attr : {"name", "site", "joint", "actuator", "tendon", "objname", "body"})
    {
      add_prefix(robot, c_out, attr);
    }
  }
}

static void merge_mujoco_worldbody(const pugi::xml_node & in, pugi::xml_node & out, const std::string & robot)
{
  for(const auto & c : in.children())
  {
    auto out_c = out.append_copy(c);
    add_prefix_recursively(robot, out_c, {"name", "childclass", "class", "material", "hfield", "mesh", "target"});
  }
}

static bfs::path get_mujoco_path(const std::string & xmlFile, const pugi::xml_node & in, const char * attr)
{
  bfs::path xmlPath = bfs::path(xmlFile).parent_path();
  auto dirAttr = in.child("compiler").attribute(attr);
  if(!dirAttr)
  {
    return xmlPath;
  }
  bfs::path dir = bfs::path(dirAttr.value());
  if(dir.is_absolute())
  {
    return dir;
  }
  else
  {
    return bfs::absolute(xmlPath / dir);
  }
}

static void merge_mujoco_model(const std::string & robot, const std::string & xmlFile, pugi::xml_node & out)
{
  pugi::xml_document in;
  if(!in.load_file(xmlFile.c_str()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to load {}", xmlFile);
  }
  auto root = in.child("mujoco");
  if(!root)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No mujoco root node in {}", xmlFile);
  }
  /** Merge compiler flags */
  {
    auto compiler_out = get_child_or_create(out, "compiler");
    merge_mujoco_compiler(xmlFile, root.child("compiler"), compiler_out);
  }
  /** Merge size attributes */
  {
    auto size_out = get_child_or_create(out, "size");
    merge_mujoco_size(root.child("size"), size_out);
  }
  /** Merge option flags */
  {
    auto option_out = get_child_or_create(out, "option");
    merge_mujoco_option(xmlFile, root.child("option"), option_out);
  }
  /** Merge defaults */
  {
    auto default_out = get_child_or_create(out, "default");
    merge_mujoco_default(xmlFile, root.child("default"), default_out, robot);
  }
  /** Merge visual */
  {
    auto visual_out = get_child_or_create(out, "visual");
    merge_mujoco_visual(xmlFile, root.child("visual"), visual_out);
  }
  /** Merge asset */
  {
    auto meshPath = get_mujoco_path(xmlFile, root, "meshdir");
    auto texturePath = get_mujoco_path(xmlFile, root, "texturedir");
    auto asset_out = get_child_or_create(out, "asset");
    merge_mujoco_asset(root.child("asset"), asset_out, meshPath, texturePath, robot);
  }
  /** Merge contact */
  {
    auto contact_out = get_child_or_create(out, "contact");
    merge_mujoco_contact(root.child("contact"), contact_out, robot);
  }
  /** Merge actuator */
  {
    auto actuator_out = get_child_or_create(out, "actuator");
    merge_mujoco_actuator(root.child("actuator"), actuator_out, robot);
  }
  /** Merge sensor */
  {
    auto sensor_out = get_child_or_create(out, "sensor");
    merge_mujoco_sensor(root.child("sensor"), sensor_out, robot);
  }
  /** Merge equality */
  {
    auto equality_out = get_child_or_create(out, "equality");
    merge_mujoco_equality(root.child("equality"), equality_out, robot);
  }
  /** Merge tendon */
  {
    auto tendon_out = get_child_or_create(out, "tendon");
    merge_mujoco_tendon(root.child("tendon"), tendon_out, robot);
  }
  /** Merge worldbody */
  {
    auto worldbody_out = get_child_or_create(out, "worldbody");
    merge_mujoco_worldbody(root.child("worldbody"), worldbody_out, robot);
  }
}

static void get_joint_names(const pugi::xml_node & in,
                            const std::string & prefix,
                            std::vector<std::string> & joints,
                            std::string & free_joint)
{
  auto prefixed = [&prefix](const char * name) -> std::string {
    if(prefix.size())
    {
      return fmt::format("{}_{}", prefix, name);
    }
    return name;
  };
  for(const auto & j : in.children("joint"))
  {
    auto type_attr = j.attribute("type");
    std::string type = type_attr ? type_attr.value() : "hinge";
    if(type != "free")
    {
      joints.push_back(prefixed(j.attribute("name").value()));
    }
    else
    {
      free_joint = prefixed(j.attribute("name").value());
    }
  }
  auto mj_free = in.child("freejoint");
  if(mj_free)
  {
    free_joint = prefixed(mj_free.attribute("name").value());
  }
  for(const auto & c : in.children("body"))
  {
    get_joint_names(c, prefix, joints, free_joint);
  }
}

static void get_motor_names(const pugi::xml_node & in,
                            const std::string & prefix,
                            const std::vector<std::string> & joints,
                            std::vector<std::string> & motors,
                            std::vector<std::string> & pos_acts,
                            std::vector<std::string> & vel_acts)
{
  auto joint_to_act = [&](const char * type, std::vector<std::string> & acts) {
    std::unordered_map<std::string, std::string> joint_to_act;
    for(const auto & m : in.children(type))
    {
      std::string name = m.attribute("name").value();
      std::string joint = m.attribute("joint").value();
      if(prefix.size())
      {
        name = fmt::format("{}_{}", prefix, name);
        joint = fmt::format("{}_{}", prefix, joint);
      }
      joint_to_act[joint] = name;
    }
    for(const auto & j : joints)
    {
      auto it = joint_to_act.find(j);
      if(it == joint_to_act.end())
      {
        acts.push_back("");
      }
      else
      {
        acts.push_back(it->second);
      }
    }
  };
  joint_to_act("motor", motors);
  joint_to_act("position", pos_acts);
  joint_to_act("velocity", vel_acts);
}

static void mj_object_from_xml(const std::string & name, const std::string & xmlFile, MjObject & object)
{
  char error[1000] = "Could not load XML model";
  auto model = mj_loadXML(xmlFile.c_str(), nullptr, error, 1000);
  if(!model)
  {
    mc_rtc::log::error_and_throw("Failed to load MuJoCo model at {}\nError: {}", xmlFile, error);
  }
  if(model->nbody < 2)
  {
    mc_rtc::log::error_and_throw("Model of {} (loaded from {}) does not have a body beside worldbody", name, xmlFile);
  }
  auto root_body = mj_id2name(model, mjOBJ_BODY, 1);
  if(!root_body)
  {
    mc_rtc::log::error_and_throw("First body in model of {} (loaded from {} root body) does not have a name", name,
                                 xmlFile);
  }
  object.root_body = fmt::format("{}_{}", name, root_body);
  if(model->nq > 0)
  {
    auto root_joint = mj_id2name(model, mjOBJ_JOINT, 0);
    object.root_joint = fmt::format("{}_{}", name, root_joint ? root_joint : "");
    object.root_joint_type = static_cast<mjtJoint>(model->jnt_type[0]);
  }
  object.nq = model->nq;
  object.ndof = model->nv;
  mj_deleteModel(model);
}

static MjRobot mj_robot_from_xml(const std::string & name, const std::string & xmlFile, const std::string & prefix = "")
{
  MjRobot out;
  out.name = name;
  out.prefix = prefix;
  pugi::xml_document in;
  if(!in.load_file(xmlFile.c_str()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to load {}", xmlFile);
  }
  auto root = in.child("mujoco");
  if(!root)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No mujoco root node in {}", xmlFile);
  }
  auto root_body = root.child("worldbody").child("body");
  if(root_body)
  {
    out.root_body = root_body.attribute("name").value();
    if(prefix.size())
    {
      out.root_body = fmt::format("{}_{}", prefix, out.root_body);
    }
  }
  get_joint_names(root.child("worldbody"), prefix, out.mj_jnt_names, out.root_joint);
  get_motor_names(root.child("actuator"), prefix, out.mj_jnt_names, out.mj_mot_names, out.mj_pos_act_names,
                  out.mj_vel_act_names);
  return out;
}

std::string merge_mujoco_models(const std::map<std::string, std::string> & mujocoObjects,
                                const std::map<std::string, std::string> & mcrtcObjects,
                                std::vector<MjObject> & mjObjects,
                                std::vector<MjRobot> & mjRobots)
{
  mjRobots.clear();
  std::string outFile = (bfs::temp_directory_path() / bfs::unique_path("mc_mujoco_%%%%-%%%%-%%%%-%%%%.xml")).string();
  pugi::xml_document out_doc;
  auto out = out_doc.append_child("mujoco");
  out.append_attribute("model").set_value("mc_mujoco");
  for(const auto & [name, xmlFile] : mujocoObjects)
  {
    merge_mujoco_model(name, xmlFile, out);
    // FIXME This is required pre-C++20 to capture structured binding value
    const auto & name2 = name;
    auto it = std::find_if(mjObjects.begin(), mjObjects.end(), [&](const auto & obj) { return obj.name == name2; });
    if(it == mjObjects.end())
    {
      mc_rtc::log::error_and_throw(
          "merge_mujoco_models given an object to load that's not in mjObjects, this should not happen");
    }
    mj_object_from_xml(name, xmlFile, *it);
  }
  for(const auto & [name, xmlFile] : mcrtcObjects)
  {
    merge_mujoco_model(name, xmlFile, out);
    mjRobots.push_back(mj_robot_from_xml(name, xmlFile, name));
  }
  {
    std::ofstream ofs(outFile);
    out_doc.save(ofs, "    ");
  }
  mc_rtc::log::info("[mc_mujoco] MuJoCo scene loaded from {}", outFile);
  return outFile;
}

} // namespace mc_mujoco
