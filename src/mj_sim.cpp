#include "mj_sim_impl.h"
#include "mj_utils.h"

#include <cassert>
#include <chrono>
#include <type_traits>

#include "MujocoClient.h"
#include "config.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <mc_rtc/version.h>

#ifdef USE_UI_ADAPTER
#  include "our_glfw_adapter.h"
#endif

namespace mc_mujoco
{

double MjRobot::PD(size_t jnt_id, double q_ref, double q, double qdot_ref, double qdot)
{
  double p_error = q_ref - q;
  double v_error = qdot_ref - qdot;
  double ret = (kp[jnt_id] * p_error + kd[jnt_id] * v_error);
  return ret;
}

/* Load PD gains from file (taken from RobotHardware/robot.cpp) */
bool MjRobot::loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints)
{
  std::ifstream strm(path_to_pd.c_str());
  if(!strm.is_open())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Cannot open PD gains file for {} at {}", name,
                                                     path_to_pd);
  }

  size_t num_joints = joints.size();
  if(!num_joints)
  {
    return false;
  }
  std::vector<double> default_pgain(num_joints, 0);
  std::vector<double> default_dgain(num_joints, 0);
  for(int i = 0; i < num_joints; i++)
  {
    std::string str;
    bool getlinep;
    while((getlinep = !!(std::getline(strm, str))))
    {
      if(str.empty())
      {
        continue;
      }
      if(str[0] == '#')
      {
        continue;
      }
      double tmp;
      std::istringstream sstrm(str);
      sstrm >> tmp;
      default_pgain[i] = tmp;
      if(sstrm.eof()) break;

      sstrm >> tmp;
      default_dgain[i] = tmp;
      if(sstrm.eof()) break;
      break;
    }
    if(!getlinep)
    {
      if(i < num_joints)
      {
        mc_rtc::log::error(
            "[mc_mujoco] loadGain error: size of gains reading from file ({}) does not match size of joints",
            path_to_pd);
      }
      break;
    }
  }

  strm.close();
  mc_rtc::log::info("[mc_mujoco] Gains for {}", name);
  for(unsigned int i = 0; i < num_joints; i++)
  {
    mc_rtc::log::info("[mc_mujoco] {}, pgain = {}, dgain = {}", joints[i], default_pgain[i], default_dgain[i]);
    // push to kp and kd
    default_kp.push_back(default_pgain[i]);
    default_kd.push_back(default_dgain[i]);
    kp.push_back(default_pgain[i]);
    kd.push_back(default_dgain[i]);
  }
  return true;
}

MjSimImpl::MjSimImpl(const MjConfiguration & config)
: controller(std::make_unique<mc_control::MCGlobalController>(config.mc_config)), config(config)
{
  auto get_robot_cfg_path_local = [&](const std::string & robot_name)
  { return bfs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path_global = [&](const std::string & robot_name)
  { return bfs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path = [&](const std::string & robot_name) -> std::string
  {
    if(bfs::exists(get_robot_cfg_path_local(robot_name)))
    {
      return get_robot_cfg_path_local(robot_name).string();
    }
    if(bfs::exists(get_robot_cfg_path_global(robot_name)))
    {
      return get_robot_cfg_path_global(robot_name).string();
    }
    return "";
  };

  /** Map between name and xml file path of objects specified in mujoco config **/
  std::map<std::string, std::string> mjObjects;
  /** Map between name and xml file path of objects specified in mc-rtc config **/
  std::map<std::string, std::string> mcObjects;
  /** Map between name and pdgains file path of objects specified in mc-rtc config **/
  std::map<std::string, std::string> pdGainsFiles;

  // load all robots named in mujoco config
  auto mc_mujoco_cfg_path = fmt::format("{}/mc_mujoco.yaml", USER_FOLDER);
  auto mc_mujoco_cfg = [&mc_mujoco_cfg_path]() -> mc_rtc::Configuration
  {
    if(bfs::exists(mc_mujoco_cfg_path))
    {
      return {mc_mujoco_cfg_path};
    }
    return {};
  }();
  auto config_objects = mc_mujoco_cfg("objects", std::map<std::string, mc_rtc::Configuration>{});
  for(const auto & co : config_objects)
  {
    MjObject object;
    object.name = static_cast<std::string>(co.first);
    object.init_pose = static_cast<sva::PTransformd>(co.second("init_pos", sva::PTransformd::Identity()));
    objects.push_back(object);

    std::string module = co.second("module");
    auto object_cfg_path = get_robot_cfg_path(module);
    if(object_cfg_path.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_mujoco] Module ({}) cannot be found at for object {}.\nTried:\n- {}\n- {}", module, co.first,
          get_robot_cfg_path_local(module).string(), get_robot_cfg_path_global(module).string());
    }
    auto object_cfg = mc_rtc::Configuration(object_cfg_path);
    if(!object_cfg.has("xmlModelPath"))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath in {}", object_cfg_path);
    }
    std::string xmlFile = static_cast<std::string>(object_cfg("xmlModelPath"));
    mjObjects[object.name] = xmlFile;
    if(!bfs::exists(xmlFile))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {} for {}", xmlFile,
                                                       co.first);
    }
  }

  // load all robots named in mc-rtc config
  for(const auto & r : controller->robots())
  {
    const auto & robot_cfg_path = get_robot_cfg_path(r.module().name);
    if(!robot_cfg_path.empty())
    {
      auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);

      auto main_robot_params = [&]() -> std::vector<std::string>
      {
        auto main_robot_cfg = robot_cfg.find("MainRobot");
        if(!main_robot_cfg)
        {
          return {"JVRC1"};
        }
        if(main_robot_cfg->isArray())
        {
          return main_robot_cfg->operator std::vector<std::string>();
        }
        if(main_robot_cfg->isObject())
        {
          auto module_cfg = (*main_robot_cfg)("module");
          if(module_cfg.isArray())
          {
            return module_cfg.operator std::vector<std::string>();
          }
          return {module_cfg.operator std::string()};
        }
        return {main_robot_cfg->operator std::string()};
      }();

      const auto & main_robot_name = main_robot_params[0];
      auto setObjectXML = [&](const std::string & xmlFile)
      {
        std::string pdGainsPath = "";
        mcObjects[r.name()] = xmlFile;
        if(!main_robot_name.empty() && robot_cfg.find(main_robot_name)
           && robot_cfg(main_robot_name).find("pdGainsPath"))
        {
          pdGainsPath = robot_cfg(main_robot_name)("pdGainsPath", std::string(""));
        }
        else if(robot_cfg.find(r.name().c_str()) && robot_cfg(r.name().c_str()).find("pdGainsPath")
                && !robot_cfg(r.name().c_str())("pdGainsPath", std::string("")).empty())
        {
          pdGainsPath = robot_cfg(r.name().c_str())("pdGainsPath", std::string(""));
        }
        else if(robot_cfg.find("pdGainsPath"))
        {
          pdGainsPath = robot_cfg("pdGainsPath", std::string(""));
        }

        if(!bfs::exists(xmlFile))
        {
          mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {} for {}",
                                                           xmlFile, r.name());
        }

        pdGainsFiles[r.name()] = pdGainsPath;
      };

      if(!robot_cfg.has("xmlModelPath") && (!robot_cfg.has(r.name()) || robot_cfg.has(main_robot_name)))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath in {}", robot_cfg_path);
      }
      else if(robot_cfg.has(main_robot_name))
      {
        setObjectXML(static_cast<std::string>(robot_cfg(main_robot_name)("xmlModelPath")));
      }
      else if(robot_cfg.has(r.name()))
      {
        setObjectXML(static_cast<std::string>(robot_cfg(r.name())("xmlModelPath")));
      }
      else
      {
        setObjectXML(static_cast<std::string>(robot_cfg("xmlModelPath")));
      }
    }
  }

  // load MuJoCo plugins
  loadPlugins(mc_mujoco_cfg);

  // initial mujoco here and load XML model
  bool initialized = mujoco_init(this, mjObjects, mcObjects);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }

  // read PD gains from file
  for(size_t i = 0; i < robots.size(); ++i)
  {
    auto & r = robots[i];
    bool has_motor =
        std::any_of(r.mj_mot_names.begin(), r.mj_mot_names.end(), [](const std::string & m) { return !m.empty(); });
    const auto & robot = controller->robot(r.name);
    if(robot.mb().nrDof() == 0 || (robot.mb().nrDof() == 6 && robot.mb().joint(0).dof() == 6) || !has_motor)
    {
      continue;
    }
    if(!bfs::exists(pdGainsFiles[r.name]))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains file for {} cannot be found at {}", r.name,
                                                       pdGainsFiles[r.name]);
    }
    r.loadGain(pdGainsFiles[r.name], controller->robots().robot(r.name).module().ref_joint_order());
  }

  if(config.with_visualization)
  {
    mujoco_create_window(this);
    if(config.with_mc_rtc_gui)
    {
      client = std::make_unique<MujocoClient>();
    }
  }
  mc_rtc::log::info("[mc_mujoco] Initialized successful.");
}

void MjSimImpl::cleanup()
{
  mujoco_cleanup(this);
}

void MjObject::initialize(mjModel * model)
{
  if(!root_body.empty())
  {
    root_body_id = mj_name2id(model, mjOBJ_BODY, root_body.c_str());
  }
  if(!root_joint.empty())
  {
    auto root_joint_id = mj_name2id(model, mjOBJ_JOINT, root_joint.c_str());
    root_qpos_idx = model->jnt_qposadr[root_joint_id];
    root_qvel_idx = model->jnt_dofadr[root_joint_id];
  }
}

void MjRobot::initialize(mjModel * model, const mc_rbdyn::Robot & robot)
{
  mj_jnt_ids.resize(0);
  for(const auto & j : mj_jnt_names)
  {
    mj_jnt_ids.push_back(mj_name2id(model, mjOBJ_JOINT, j.c_str()));
  }
  auto fill_actuator_ids = [&](const std::vector<std::string> & names, std::vector<int> & ids)
  {
    ids.resize(0);
    for(const auto & n : names)
    {
      if(!n.empty())
      {
        ids.push_back(mj_name2id(model, mjOBJ_ACTUATOR, n.c_str()));
      }
      else
      {
        ids.push_back(-1);
      }
    }
  };
  fill_actuator_ids(mj_mot_names, mj_mot_ids);
  fill_actuator_ids(mj_pos_act_names, mj_pos_act_ids);
  fill_actuator_ids(mj_vel_act_names, mj_vel_act_ids);
  if(!root_body.empty())
  {
    root_body_id = mj_name2id(model, mjOBJ_BODY, root_body.c_str());
  }
  if(!root_joint.empty())
  {
    auto root_joint_id = mj_name2id(model, mjOBJ_JOINT, root_joint.c_str());
    root_qpos_idx = model->jnt_qposadr[root_joint_id];
    root_qvel_idx = model->jnt_dofadr[root_joint_id];
  }
  auto init_sensor_id = [&](const char * mj_name, const char * mc_name, const std::string & sensor_name,
                            const char * suffix, mjtSensor type, std::unordered_map<std::string, int> & mapping)
  {
    auto mj_sensor = prefixed(fmt::format("{}_{}", sensor_name, suffix));
    auto sensor_id = mujoco_get_sensor_id(*model, mj_sensor, type);
    if(sensor_id == -1)
    {
      mc_rtc::log::error("[mc_mujoco] No MuJoCo {} for {} {} in {}, expected to find a {} named {}", mj_name,
                         sensor_name, mc_name, name, mj_name, mj_sensor);
    }
    mapping[sensor_name] = sensor_id;
  };
  for(const auto & fs : robot.module().forceSensors())
  {
    wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
    init_sensor_id("force sensor", "force sensor", fs.name(), "fsensor", mjSENS_FORCE, mc_fs_to_mj_fsensor_id);
    init_sensor_id("torque sensor", "force sensor", fs.name(), "tsensor", mjSENS_TORQUE, mc_fs_to_mj_tsensor_id);
  }
  for(const auto & bs : robot.bodySensors())
  {
    if(bs.name() == "FloatingBase" || bs.name().empty())
    {
      continue;
    }
    gyros[bs.name()] = Eigen::Vector3d::Zero();
    accelerometers[bs.name()] = Eigen::Vector3d::Zero();
    init_sensor_id("gyro sensor", "body sensor", bs.name(), "gyro", mjSENS_GYRO, mc_bs_to_mj_gyro_id);
    init_sensor_id("accelerometer sensor", "body sensor", bs.name(), "accelerometer", mjSENS_ACCELEROMETER,
                   mc_bs_to_mj_accelerometer_id);
  }
  reset(robot);
}

void MjRobot::reset(const mc_rbdyn::Robot & robot)
{
  const auto & mbc = robot.mbc();
  const auto & rjo = robot.module().ref_joint_order();
  if(rjo.size() != mj_jnt_names.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_mujoco] Missmatch in model for {}, reference joint order has {} joints but MuJoCo models has {} joints",
        name, rjo.size(), mj_jnt_names.size());
  }
  mj_to_mbc.resize(0);
  mj_prev_ctrl_q.resize(0);
  mj_prev_ctrl_alpha.resize(0);
  mj_prev_ctrl_jointTorque.resize(0);
  mj_jnt_to_rjo.resize(0);
  mj_to_mbc.resize(0);
  encoders = std::vector<double>(rjo.size(), 0.0);
  alphas = std::vector<double>(rjo.size(), 0.0);
  torques = std::vector<double>(rjo.size(), 0.0);
  for(const auto & mj_jn : mj_jnt_names)
  {
    const auto & jn = [&]()
    {
      if(!prefix.empty())
      {
        return mj_jn.substr(prefix.size() + 1);
      }
      return mj_jn;
    }();
    auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
    int rjo_idx = -1;
    if(rjo_it != rjo.end())
    {
      rjo_idx = std::distance(rjo.begin(), rjo_it);
    }
    mj_jnt_to_rjo.push_back(rjo_idx);
    if(robot.hasJoint(jn))
    {
      auto jIndex = robot.jointIndexByName(jn);
      mj_to_mbc.push_back(jIndex);
      if(robot.mb().joint(jIndex).dof() != 1)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[mc_mujoco] Only support revolute and prismatic joint for control");
      }
      mj_prev_ctrl_q.push_back(robot.mbc().q[jIndex][0]);
      mj_prev_ctrl_alpha.push_back(robot.mbc().alpha[jIndex][0]);
      mj_prev_ctrl_jointTorque.push_back(robot.mbc().jointTorque[jIndex][0]);
      if(rjo_idx != -1)
      {
        encoders[rjo_idx] = mj_prev_ctrl_q.back();
        alphas[rjo_idx] = mj_prev_ctrl_alpha.back();
        torques[rjo_idx] = mj_prev_ctrl_jointTorque.back();
      }
    }
    else
    {
      mj_to_mbc.push_back(-1);
    }
  }
  mj_ctrl = std::vector<double>(mj_prev_ctrl_q.size(), 0.0);
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;
  mj_next_ctrl_jointTorque = mj_prev_ctrl_jointTorque;

  // reset the PD gains to default values
  kp = default_kp;
  kd = default_kd;
}

template<typename T>
void MjSimImpl::setPosW(const T & robot, const sva::PTransformd & pos)
{
  const auto & t = pos.translation();
  Eigen::Quaterniond q = Eigen::Quaterniond(pos.rotation()).inverse();
  if(robot.root_qpos_idx != -1 && robot.root_joint_type == mjJNT_FREE)
  {
    data->qpos[robot.root_qpos_idx + 0] = t.x();
    data->qpos[robot.root_qpos_idx + 1] = t.y();
    data->qpos[robot.root_qpos_idx + 2] = t.z();
    data->qpos[robot.root_qpos_idx + 3] = q.w();
    data->qpos[robot.root_qpos_idx + 4] = q.x();
    data->qpos[robot.root_qpos_idx + 5] = q.y();
    data->qpos[robot.root_qpos_idx + 6] = q.z();
    // push linear/angular velocities
    mju_zero3(&data->qvel[robot.root_qvel_idx]);
    mju_zero3(&data->qvel[robot.root_qvel_idx + 3]);
  }
  else if(robot.root_body_id != -1)
  {
    model->body_pos[3 * robot.root_body_id + 0] = t.x();
    model->body_pos[3 * robot.root_body_id + 1] = t.y();
    model->body_pos[3 * robot.root_body_id + 2] = t.z();
    model->body_quat[4 * robot.root_body_id + 0] = q.w();
    model->body_quat[4 * robot.root_body_id + 1] = q.x();
    model->body_quat[4 * robot.root_body_id + 2] = q.y();
    model->body_quat[4 * robot.root_body_id + 3] = q.z();
  }
}

sva::PTransformd MjSimImpl::getObjectPosW(const std::string & object) const
{
  auto it = std::find_if(objects.begin(), objects.end(), [&](const auto & o) { return o.name == object; });
  if(it == objects.end())
  {
    mc_rtc::log::error_and_throw("Requested position of object {} which is not in this simulation", object);
  }
  const auto & o = *it;
  if(o.root_qpos_idx != -1 && o.root_joint_type == mjJNT_FREE)
  {
    Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(&data->qpos[o.root_qpos_idx]);
    // Note: no map here because Eigen::Map wants x, y, z, w
    Eigen::Quaterniond q;
    q.w() = data->qpos[o.root_qpos_idx + 3];
    q.x() = data->qpos[o.root_qpos_idx + 4];
    q.y() = data->qpos[o.root_qpos_idx + 5];
    q.z() = data->qpos[o.root_qpos_idx + 6];
    return {q.inverse(), t};
  }
  if(o.root_body_id != -1)
  {
    Eigen::Vector3d t = Eigen::Map<Eigen::Vector3d>(&model->body_pos[3 * o.root_body_id]);
    Eigen::Quaterniond q;
    q.w() = model->body_quat[4 * o.root_body_id + 0];
    q.x() = model->body_quat[4 * o.root_body_id + 1];
    q.y() = model->body_quat[4 * o.root_body_id + 2];
    q.z() = model->body_quat[4 * o.root_body_id + 3];
    return {q.inverse(), t};
  }
  mc_rtc::log::error_and_throw("Cannot retrieve the position of object {} in simulation", object);
}

void MjSimImpl::setSimulationInitialState()
{
  if(controller)
  {
    for(auto & o : objects)
    {
      o.initialize(model);
      setPosW(o, o.init_pose);
    }

    for(auto & r : robots)
    {
      const auto & robot = controller->robots().robot(r.name);
      r.initialize(model, robot);
      setPosW(r, robot.posW());
      for(size_t i = 0; i < r.mj_jnt_ids.size(); ++i)
      {
        if(r.mj_jnt_to_rjo[i] == -1)
        {
          continue;
        }
        data->qpos[model->jnt_qposadr[r.mj_jnt_ids[i]]] = r.encoders[r.mj_jnt_to_rjo[i]];
        data->qvel[model->jnt_dofadr[r.mj_jnt_ids[i]]] = r.alphas[r.mj_jnt_to_rjo[i]];
      }
    }
  }
  mj_forward(model, data);
}

void MjSimImpl::setObjectPosW(const std::string & object, const sva::PTransformd & pt)
{
  for(const auto & o : objects)
  {
    if(o.name == object)
    {
      setPosW(o, pt);
      return;
    }
  }
}

void MjSimImpl::setRobotPosW(const std::string & robot, const sva::PTransformd & pt)
{
  for(const auto & r : robots)
  {
    if(r.name == robot)
    {
      setPosW(r, pt);
      return;
    }
  }
}

void MjSimImpl::makeDatastoreCalls()
{
  auto & ds = controller->controller().datastore();
  for(auto & o : objects)
  {
    ds.make_call(o.name + "::SetPosW", [this, name = o.name](const sva::PTransformd & pt) { setObjectPosW(name, pt); });
  }
  for(auto & r : robots)
  {
    ds.make_call(r.name + "::SetPosW", [this, name = r.name](const sva::PTransformd & pt) { setRobotPosW(name, pt); });
    // make_call for setting pd gains (for all joints)
    ds.make_call(r.name + "::SetPDGains",
                 [this, &r](const std::vector<double> & p_vec, const std::vector<double> & d_vec)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   if(p_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. p_vec size({})!=ref_joint_order size({})",
                                          r.name, p_vec.size(), rjo.size());
                     return false;
                   }
                   if(d_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. d_vec size({})!=ref_joint_order size({})",
                                          r.name, d_vec.size(), rjo.size());
                     return false;
                   }
                   r.kp = p_vec;
                   r.kd = d_vec;
                   return true;
                 });

    // make_call for setting pd gains (by name)
    ds.make_call(r.name + "::SetPDGainsByName",
                 [this, &r](const std::string & jn, double p, double d)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
                   if(rjo_it == rjo.end())
                   {
                     mc_rtc::log::warning(
                         "[mc_mujoco] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.", r.name, jn);
                     return false;
                   }
                   int rjo_idx = std::distance(rjo.begin(), rjo_it);
                   r.kp[rjo_idx] = p;
                   r.kd[rjo_idx] = d;
                   return true;
                 });

    // make_call for reading pd gains (for all joints)
    ds.make_call(r.name + "::GetPDGains",
                 [this, &r](std::vector<double> & p_vec, std::vector<double> & d_vec)
                 {
                   p_vec.resize(0);
                   d_vec.resize(0);
                   p_vec = r.kp;
                   d_vec = r.kd;
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   if(p_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. p_vec size({})!=ref_joint_order size({})",
                                          r.name, p_vec.size(), rjo.size());
                     return false;
                   }
                   if(d_vec.size() != rjo.size())
                   {
                     mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. d_vec size({})!=ref_joint_order size({})",
                                          r.name, d_vec.size(), rjo.size());
                     return false;
                   }
                   return true;
                 });

    // make_call for reading pd gains (by name)
    ds.make_call(r.name + "::GetPDGainsByName",
                 [this, &r](const std::string & jn, double & p, double & d)
                 {
                   const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
                   auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
                   if(rjo_it == rjo.end())
                   {
                     mc_rtc::log::warning(
                         "[mc_mujoco] {}::GetPDGainsByName failed. Joint {} not found in ref_joint_order.", r.name, jn);
                     return false;
                   }
                   int rjo_idx = std::distance(rjo.begin(), rjo_it);
                   p = r.kp[rjo_idx];
                   d = r.kd[rjo_idx];
                   return true;
                 });
  }
}

void MjSimImpl::startSimulation()
{
  if(!config.with_controller)
  {
    setSimulationInitialState();
    controller.reset();
    return;
  }

  makeDatastoreCalls();

  // get sim timestep and set the frameskip parameter
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  for(auto & r : robots)
  {
    r.initialize(model, controller->robot(r.name));
    controller->setEncoderValues(r.name, r.encoders);
  }
  for(const auto & r : robots)
  {
    init_qs_[r.name] = controller->robot(r.name).encoderValues();
    init_pos_[r.name] = controller->controller().robot(r.name).posW();
  }
  controller->init(init_qs_, init_pos_);
  controller->running = true;
  setSimulationInitialState();
}

void MjRobot::updateSensors(mc_control::MCGlobalController * gc, mjModel * model, mjData * data)
{
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1)
    {
      continue;
    }
    encoders[mj_jnt_to_rjo[i]] = data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]];
    alphas[mj_jnt_to_rjo[i]] = data->qvel[model->jnt_dofadr[mj_jnt_ids[i]]];
  }
  for(size_t i = 0; i < mj_mot_ids.size(); ++i)
  {
    if(mj_jnt_to_rjo[i] == -1)
    {
      continue;
    }
    torques[mj_jnt_to_rjo[i]] = data->qfrc_actuator[model->jnt_dofadr[mj_jnt_ids[i]]];
  }
  if(!gc)
  {
    return;
  }
  auto & robot = gc->controller().robots().robot(name);

  // Body sensor updates
  if(root_qpos_idx != -1)
  {
    root_pos = Eigen::Map<Eigen::Vector3d>(&data->qpos[root_qpos_idx]);
    root_ori.w() = data->qpos[root_qpos_idx + 3];
    root_ori.x() = data->qpos[root_qpos_idx + 4];
    root_ori.y() = data->qpos[root_qpos_idx + 5];
    root_ori.z() = data->qpos[root_qpos_idx + 6];
    root_ori = root_ori.inverse();
    root_linvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx]);
    root_angvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx + 3]);
    root_linacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx]);
    root_angacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx + 3]);
    if(robot.hasBodySensor("FloatingBase"))
    {
      gc->setSensorPositions(name, {{"FloatingBase", root_pos}});
      gc->setSensorOrientations(name, {{"FloatingBase", root_ori}});
      gc->setSensorLinearVelocities(name, {{"FloatingBase", root_linvel}});
      gc->setSensorAngularVelocities(name, {{"FloatingBase", root_angvel}});
      gc->setSensorLinearAccelerations(name, {{"FloatingBase", root_linacc}});
      // FIXME Not implemented in mc_rtc
      // gc->setSensorAngularAccelerations(name, {{"FloatingBase", root_angacc}});
    }
  }

  // Gyro update
  for(auto & gyro : gyros)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_gyro_id[gyro.first], gyro.second.data());
  }
  gc->setSensorAngularVelocities(name, gyros);

  // Accelerometers update
  for(auto & accelerometer : accelerometers)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_accelerometer_id[accelerometer.first],
                          accelerometer.second.data());
  }
  gc->setSensorLinearAccelerations(name, accelerometers);

  // Force sensor update
  for(auto & fs : wrenches)
  {
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_fsensor_id[fs.first], fs.second.force().data());
    mujoco_get_sensordata(*model, *data, mc_fs_to_mj_tsensor_id[fs.first], fs.second.couple().data());
    fs.second *= -1;
  }
  gc->setWrenches(name, wrenches);

  // Joint sensor updates
  gc->setEncoderValues(name, encoders);
  gc->setEncoderVelocities(name, alphas);
  gc->setJointTorques(name, torques);
}

void MjSimImpl::updateData()
{
  for(auto & r : robots)
  {
    r.updateSensors(controller.get(), model, data);
  }
}

void MjRobot::updateControl(const mc_rbdyn::Robot & robot)
{
  mj_prev_ctrl_q = mj_next_ctrl_q;
  mj_prev_ctrl_alpha = mj_next_ctrl_alpha;
  mj_prev_ctrl_jointTorque = mj_next_ctrl_jointTorque;
  size_t ctrl_idx = 0;
  for(size_t i = 0; i < mj_to_mbc.size(); ++i)
  {
    auto jIndex = mj_to_mbc[i];
    if(jIndex != -1)
    {
      mj_next_ctrl_q[ctrl_idx] = robot.mbc().q[jIndex][0];
      mj_next_ctrl_alpha[ctrl_idx] = robot.mbc().alpha[jIndex][0];
      mj_next_ctrl_jointTorque[ctrl_idx] = robot.mbc().jointTorque[jIndex][0];
      ctrl_idx++;
    }
  }
}

void MjRobot::sendControl(const mjModel & model,
                          mjData & data,
                          size_t interp_idx,
                          size_t frameskip_,
                          bool torque_control)
{
  for(size_t i = 0; i < mj_ctrl.size(); ++i)
  {
    auto mot_id = mj_mot_ids[i];
    auto pos_act_id = mj_pos_act_ids[i];
    auto vel_act_id = mj_vel_act_ids[i];
    auto rjo_id = mj_jnt_to_rjo[i];
    if(rjo_id == -1)
    {
      continue;
    }
    // compute desired q using interpolation
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];
    // compute desired alpha using interpolation
    double alpha_ref = (interp_idx + 1) * (mj_next_ctrl_alpha[i] - mj_prev_ctrl_alpha[i]) / frameskip_;
    alpha_ref += mj_prev_ctrl_alpha[i];
    // compute desired jointTorque using interpolation
    double torque_ref = (interp_idx + 1) * (mj_next_ctrl_jointTorque[i] - mj_prev_ctrl_jointTorque[i]) / frameskip_;
    torque_ref += mj_prev_ctrl_jointTorque[i];
    if(mot_id != -1)
    {
      if(torque_control && torque_ref != 0)
      {
        mj_ctrl[i] = torque_ref;
      }
      else
      {
        mj_ctrl[i] = PD(rjo_id, q_ref, encoders[rjo_id], alpha_ref, alphas[rjo_id]);
      }
      double ratio = model.actuator_gear[6 * mot_id];
      data.ctrl[mot_id] = mj_ctrl[i] / ratio;
    }
    if(pos_act_id != -1)
    {
      data.ctrl[pos_act_id] = q_ref;
    }
    if(vel_act_id != -1)
    {
      data.ctrl[vel_act_id] = alpha_ref;
    }
  }
}

bool MjSimImpl::controlStep()
{
  auto interp_idx = iterCount_ % frameskip_;
  // After every frameskip iters
  if(config.with_controller && interp_idx == 0)
  {
    // run the controller
    if(!controller->run())
    {
      return true;
    }
    for(auto & r : robots)
    {
      r.updateControl(controller->robots().robot(r.name));
    }
  }
  // On each control iter
  for(auto & r : robots)
  {
    r.sendControl(*model, *data, interp_idx, frameskip_, config.torque_control);
  }
  iterCount_++;
  return false;
}

void MjSimImpl::simStep()
{
  // clear old perturbations, apply new
  mju_zero(data->xfrc_applied, 6 * model->nbody);
  mjv_applyPerturbPose(model, data, &pert, 0); // move mocap bodies only
  mjv_applyPerturbForce(model, data, &pert);

  // take one step in simulation
  // model.opt.timestep will be used here
  mj_step(model, data);

  wallclock = data->time;
}

void MjSimImpl::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                                const std::map<std::string, sva::PTransformd> & reset_pos)
{
  iterCount_ = 0;
  reset_simulation_ = false;
  if(controller)
  {
    controller->reset(reset_qs, reset_pos);
    for(auto & robot : robots)
    {
      robot.reset(controller->robot(robot.name));
    }
    controller->running = true;
  }
  mj_resetData(model, data);
  setSimulationInitialState();
  makeDatastoreCalls();
  for(auto & marker : markers)
  {
    marker.marker.pose(getObjectPosW(marker.name));
  }
}

bool MjSimImpl::stepSimulation()
{
  if(reset_simulation_)
  {
    resetSimulation(init_qs_, init_pos_);
  }
  auto start_step = clock::now();
  // Only run the GUI update if the simulation is paused
  if(config.step_by_step && rem_steps == 0)
  {
    mj_kinematics(model, data);
    if(controller)
    {
      controller->running = false;
      controller->run();
      controller->running = true;
    }
    mj_sim_start_t = start_step;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
  }
  if(iterCount_ > 0)
  {
    duration_us dt = start_step - mj_sim_start_t;
    mj_sync_delay += duration_us(1e6 * model->opt.timestep) - dt;
    mj_sim_dt[(iterCount_ - 1) % mj_sim_dt.size()] = dt.count();
  }
  mj_sim_start_t = start_step;
  auto do_step = [this, &start_step]()
  {
    {
      std::lock_guard<std::mutex> lock(rendering_mutex_);
      simStep();
    }
    updateData();
    return controlStep();
  };
  bool done = false;
  if(!config.step_by_step)
  {
    done = do_step();
  }
  if(config.step_by_step && rem_steps > 0)
  {
    // Doing 'frameskip_' steps of sim + control
    // (But controller.run() will execute only when interp_idx == 0)
    for(size_t i = 0; i < frameskip_; i++)
    {
      done = do_step() && done;
    }
    rem_steps--;
  }
  if(config.sync_real_time)
  {
    std::this_thread::sleep_until(start_step + duration_us(1e6 * model->opt.timestep) + mj_sync_delay);
  }
  return done;
}

void MjSimImpl::updateScene()
{
  // update scene and render
  std::lock_guard<std::mutex> lock(rendering_mutex_);
  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);

  if(client)
  {
    client->updateScene(scene);
  }

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

bool MjSimImpl::render()
{
  if(!config.with_visualization)
  {
    return true;
  }

  // mj render
#ifdef USE_UI_ADAPTER
  mjr_render(platform_ui_adapter->state().rect[0], &scene, &platform_ui_adapter->mjr_context());
#else
  mjr_render(uistate.rect[0], &scene, &context);
#endif

  // Render ImGui
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();
  ImGuiIO & io = ImGui::GetIO();
  ImGuizmo::AllowAxisFlip(false);
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
  if(client)
  {
    client->update();
#ifdef USE_UI_ADAPTER
    client->draw2D(*platform_ui_adapter);
#else
    client->draw2D(window);
#endif
    client->draw3D();
    for(auto & [name, marker] : markers)
    {
      if(marker.draw(client->view(), client->projection()) || marker.active())
      {
        setObjectPosW(name, marker.pose());
      }
    }
  }
  {
    auto right_margin = 5.0f;
    auto top_margin = 5.0f;
    auto width = io.DisplaySize.x - 2 * right_margin;
    auto height = io.DisplaySize.y - 2 * top_margin;
    ImGui::SetNextWindowPos({0.8f * width - right_margin, top_margin}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({0.2f * width, 0.3f * height}, ImGuiCond_FirstUseEver);
#if mjVERSION_HEADER <= 210
    ImGui::Begin(fmt::format("mc_mujoco (MuJoCo {})", mj_version()).c_str());
#else
    ImGui::Begin(fmt::format("mc_mujoco (MuJoCo {})", mj_versionString()).c_str());
#endif
    size_t nsamples = std::min(mj_sim_dt.size(), iterCount_);
    mj_sim_dt_average = 0;
    for(size_t i = 0; i < nsamples; ++i)
    {
      mj_sim_dt_average += mj_sim_dt[i] / nsamples;
    }
    ImGui::Text("Average sim time: %.2fμs", mj_sim_dt_average);
    ImGui::Text("Simulation/Real time: %.2f", mj_sim_dt_average / (1e6 * model->opt.timestep));
    ImGui::Text("Wallclock time: %.2fs", wallclock);
    if(ImGui::Checkbox("Sync with real-time", &config.sync_real_time))
    {
      if(config.sync_real_time)
      {
        mj_sync_delay = duration_us(0);
      }
    }
    ImGui::Checkbox("Step-by-step", &config.step_by_step);
    if(config.step_by_step)
    {
      auto doNStepsButton = [&](size_t n, bool final_)
      {
        size_t n_ms = std::ceil(n * 1000 * (controller ? controller->timestep() : model->opt.timestep));
        if(ImGui::Button(fmt::format("+{}ms", n_ms).c_str()))
        {
          rem_steps = n;
        }
        if(!final_)
        {
          ImGui::SameLine();
        }
      };
      doNStepsButton(1, false);
      doNStepsButton(5, false);
      doNStepsButton(10, false);
      doNStepsButton(50, false);
      doNStepsButton(100, true);
    }
    auto flag_to_gui = [&](const char * label, mjtVisFlag flag)
    {
      bool show = options.flags[flag];
      if(ImGui::Checkbox(label, &show))
      {
        options.flags[flag] = show;
      }
    };
    flag_to_gui("Show contact points [C]", mjVIS_CONTACTPOINT);
    flag_to_gui("Show contact forces [F]", mjVIS_CONTACTFORCE);
    flag_to_gui("Make Transparent [T]", mjVIS_TRANSPARENT);
    flag_to_gui("Convex Hull rendering [V]", mjVIS_CONVEXHULL);
    auto group_to_checkbox = [&](size_t group, bool last)
    {
      bool show = options.geomgroup[group];
      if(ImGui::Checkbox(fmt::format("{}", group).c_str(), &show))
      {
        options.geomgroup[group] = show;
      }
      if(!last)
      {
        ImGui::SameLine();
      }
    };
    ImGui::Text("%s", fmt::format("Visible layers [0-{}]", mjNGROUP).c_str());
    for(size_t i = 0; i < mjNGROUP; ++i)
    {
      group_to_checkbox(i, i == mjNGROUP - 1);
    }
    if(ImGui::Button("Reset simulation", ImVec2(-FLT_MIN, 0.0f)))
    {
      reset_simulation_ = true;
    }
    for(const auto & o : objects)
    {
      auto it = std::find_if(markers.begin(), markers.end(), [&](const auto & m) { return m.name == o.name; });
      bool active = (it != markers.end());
      if(ImGui::Checkbox(fmt::format("Set {} position", o.name).c_str(), &active))
      {
        if(!active)
        {
          markers.erase(it);
        }
        else
        {
          markers.push_back(MjObjectMarker{o.name, {getObjectPosW(o.name), ControlAxis::ALL}});
        }
      }
    }
    ImGui::End();
  }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // swap OpenGL buffers (blocking call due to v-sync)
#ifdef USE_UI_ADAPTER
  platform_ui_adapter->SwapBuffers();
#else
  glfwSwapBuffers(window);
#endif

#ifdef USE_UI_ADAPTER
  return !platform_ui_adapter->ShouldCloseWindow();
#else
  return !glfwWindowShouldClose(window);
#endif
}

void MjSimImpl::stopSimulation() {}

void MjSimImpl::saveGUISettings()
{
  auto user_path = bfs::path(USER_FOLDER);
  if(!bfs::exists(user_path))
  {
    if(!bfs::create_directories(user_path))
    {
      mc_rtc::log::critical("Failed to create the user directory: {}. GUI configuration will not be saved",
                            user_path.string());
      return;
    }
  }

  auto config_path = fmt::format("{}/mc_mujoco.yaml", USER_FOLDER);
  auto config = [&]() -> mc_rtc::Configuration
  {
    if(bfs::exists(config_path))
    {
      return {config_path};
    }
    return {};
  }();

  auto camera_c = config.add("camera");
  camera_c.add("type", camera.type);
  camera_c.add("fixedcamid", camera.fixedcamid);
  camera_c.add("trackbodyid", camera.trackbodyid);
  auto lookat = camera_c.array("lookat", 3);
  for(size_t i = 0; i < 3; ++i)
  {
    lookat.push(camera.lookat[i]);
  }
  camera_c.add("distance", camera.distance);
  camera_c.add("azimuth", camera.azimuth);
  camera_c.add("elevation", camera.elevation);
  auto visualize_c = config.add("visualize");
  visualize_c.add("collisions", static_cast<bool>(options.geomgroup[0]));
  visualize_c.add("visuals", static_cast<bool>(options.geomgroup[1]));
  visualize_c.add("contact-points", static_cast<bool>(options.flags[mjVIS_CONTACTPOINT]));
  visualize_c.add("contact-forces", static_cast<bool>(options.flags[mjVIS_CONTACTFORCE]));
  visualize_c.add("contact-split", static_cast<bool>(options.flags[mjVIS_CONTACTSPLIT]));
  config.save(config_path);
  mc_rtc::log::success("[mc_mujoco] Configuration saved to {}", config_path);
}

void MjSimImpl::loadPlugins(const mc_rtc::Configuration & mc_mujoco_cfg) const
{
  // print built-in plugins
  int nplugin = mjp_pluginCount();
  if(nplugin)
  {
    mc_rtc::log::info("[mc_mujoco] Built-in plugins");
    for(int i = 0; i < nplugin; ++i)
    {
      mc_rtc::log::info("  - {}", mjp_getPluginAtSlot(i)->name);
    }
  }

  // scan plugins from the user-specified paths
  for(const auto & plugin_path : mc_mujoco_cfg("PluginPaths", std::vector<std::string>{}))
  {
    mc_rtc::log::info("[mc_mujoco] Scan plugins in {}", plugin_path);
    mj_loadAllPluginLibraries(
        plugin_path.c_str(),
        +[](const char * filename, int first, int count)
        {
          if(count == 0)
          {
            return;
          }
          mc_rtc::log::info("[mc_mujoco] Plugins registered by library {}", filename);
          for(int i = first; i < first + count; ++i)
          {
            mc_rtc::log::info("  - {}", mjp_getPluginAtSlot(i)->name);
          }
        });
  }
}

MjSim::MjSim(const MjConfiguration & config) : impl(new MjSimImpl(config))
{
  impl->startSimulation();
}

MjSim::~MjSim()
{
  impl->cleanup();
}

bool MjSim::stepSimulation()
{
  return impl->stepSimulation();
}

void MjSim::stopSimulation()
{
  impl->stopSimulation();
}

void MjSim::updateScene()
{
  impl->updateScene();
}

void MjSim::resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                            const std::map<std::string, sva::PTransformd> & reset_pos)
{
  impl->resetSimulation(reset_qs, reset_pos);
}

bool MjSim::render()
{
  return impl->render();
}

mc_control::MCGlobalController * MjSim::controller() noexcept
{
  return impl->get_controller();
}

mjModel & MjSim::model() noexcept
{
  return *impl->model;
}

mjData & MjSim::data() noexcept
{
  return *impl->data;
}

} // namespace mc_mujoco
