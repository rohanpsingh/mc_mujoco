#include "mj_sim_impl.h"
#include "mj_utils.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <type_traits>
#include <unordered_set>

#include "MujocoClient.h"
#include "config.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include <filesystem>
namespace fs = std::filesystem;

#include <mc_rtc/version.h>

#ifdef USE_UI_ADAPTER
#  include "our_glfw_adapter.h"
#endif

// In mj_utils.h
static inline void updateVector(const std::vector<std::vector<double>> & value, Eigen::VectorXd & vec)
{
  Eigen::DenseIndex idx = 0;
  for(const auto & block : value)
  {
    Eigen::DenseIndex size = static_cast<Eigen::DenseIndex>(block.size());
    if(size == 0) continue;
    Eigen::Map<const Eigen::VectorXd> qi(block.data(), size);
    vec.segment(idx, size) = qi;
    idx += size;
  }
}

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
  { return fs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path_global = [&](const std::string & robot_name)
  { return fs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml"); };
  auto get_robot_cfg_path = [&](const std::string & robot_name) -> std::string
  {
    if(fs::exists(get_robot_cfg_path_local(robot_name)))
    {
      return get_robot_cfg_path_local(robot_name).string();
    }
    if(fs::exists(get_robot_cfg_path_global(robot_name)))
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
    if(fs::exists(mc_mujoco_cfg_path))
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
    if(!fs::exists(xmlFile))
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
        auto main_robot_cfg = controller->configuration().config.find("MainRobot");
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

        if(!fs::exists(xmlFile))
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
    if(!fs::exists(pdGainsFiles[r.name]))
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
  if(!mj_general_act_name.empty())
  {
    mj_general_act_id = mj_name2id(model, mjOBJ_ACTUATOR, mj_general_act_name.c_str());
  }
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
  std::unordered_set<std::string> gripper_active_joints;
  for(const auto & g : robot.grippers())
  {
    for(const auto & joint : g.get().activeJoints())
    {
      gripper_active_joints.insert(joint);
    }
  }
  if(mj_jnt_names.size() > rjo.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_mujoco] Missmatch in model for {}, reference joint order has {} joints but MuJoCo models has {} joints",
        name, rjo.size(), mj_jnt_names.size());
  }
  if(rjo.size() != mj_jnt_names.size())
  {
    mc_rtc::log::info("[mc_mujoco] {} uses {} controller joints for {} MuJoCo joints; unmatched MuJoCo joints will "
                      "be treated as internal/passive joints",
                      name, rjo.size(), mj_jnt_names.size());
  }
  mj_to_mbc.resize(0);
  mj_prev_ctrl_q.resize(0);
  mj_prev_ctrl_alpha.resize(0);
  mj_prev_ctrl_jointTorque.resize(0);
  mj_jnt_to_rjo.resize(0);
  mj_to_mbc.resize(0);
  mj_is_gripper_joint.resize(0);
  mj_general_act_ctrl_idx = -1;
  encoders = std::vector<double>(rjo.size(), 0.0);
  alphas = std::vector<double>(rjo.size(), 0.0);
  torques = std::vector<double>(rjo.size(), 0.0);
  tau_ext = Eigen::VectorXd::Zero(robot.mb().nrDof());
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
      mj_is_gripper_joint.push_back(gripper_active_joints.count(jn) != 0);
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
      mj_is_gripper_joint.push_back(false);
      mc_rtc::log::warning("[mc_mujoco] No matching joint in controller for MuJoCo joint {} in {}", mj_jn, name);
    }
  }
  if(mj_general_act_id != -1 && gripper_active_joints.size() == 1)
  {
    const auto prefixed_joint = prefixed(*gripper_active_joints.begin());
    auto it = std::find(mj_jnt_names.begin(), mj_jnt_names.end(), prefixed_joint);
    if(it != mj_jnt_names.end())
    {
      mj_general_act_ctrl_idx = static_cast<int>(std::distance(mj_jnt_names.begin(), it));
    }
    else
    {
      mc_rtc::log::warning("[mc_mujoco] Could not map gripper joint {} to a MuJoCo command source in {}",
                           *gripper_active_joints.begin(), name);
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
    ds.make_call(o.name + "::GetPosW", [this, name = o.name]() { return getObjectPosW(name); });
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
    controller->setEncoderVelocities(r.name, r.alphas);
    controller->setJointTorques(r.name, r.torques);
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
    // For mimic joints (no motor, driven by MuJoCo equality constraints),
    // do NOT feed back the MuJoCo-driven position. Instead keep the value
    // consistent with what the controller commanded, so the QP state in
    // runClosedLoop() is never corrupted by equality-constraint-driven motion
    // that mc-rtc does not model.
    if(mj_mot_ids[i] == -1 && mj_pos_act_ids[i] == -1 && mj_vel_act_ids[i] == -1)
    {
      // This joint has no actuator in MuJoCo — it is driven by an equality
      // constraint. Leave its encoder slot at the last commanded value so
      // the observer does not inject an inconsistent state into the QP.
      continue;
    }
    encoders[mj_jnt_to_rjo[i]] = data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]];
    alphas[mj_jnt_to_rjo[i]] = data->qvel[model->jnt_dofadr[mj_jnt_ids[i]]];
    torques[mj_jnt_to_rjo[i]] = data->qfrc_actuator[model->jnt_dofadr[mj_jnt_ids[i]]];
  }

  if(!gc)
  {
    return;
  }
  auto & robot = gc->controller().robots().robot(name);
  auto & realRobot = gc->controller().realRobots().robot(name);

  // Body sensor updates
  if(root_qpos_idx != -1)
  {
    root_pos = Eigen::Map<Eigen::Vector3d>(&data->qpos[root_qpos_idx]);
    root_ori.w() = data->qpos[root_qpos_idx + 3];
    root_ori.x() = data->qpos[root_qpos_idx + 4];
    root_ori.y() = data->qpos[root_qpos_idx + 5];
    root_ori.z() = data->qpos[root_qpos_idx + 6];
    const Eigen::Matrix3d R_0_b = root_ori.toRotationMatrix(); // MuJoCo quat = R_0_b
    root_ori = root_ori.inverse(); // keep: mc_rtc wants R^T
    root_linvel = Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx]); // world, OK
    root_angvel = R_0_b * Eigen::Map<Eigen::Vector3d>(&data->qvel[root_qvel_idx + 3]); // body -> world
    root_angacc = R_0_b * Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx + 3]); // body -> world
    root_linacc = Eigen::Map<Eigen::Vector3d>(&data->qacc[root_qvel_idx]) - root_angvel.cross(root_linvel);
    if(robot.hasBodySensor("FloatingBase"))
    {
      gc->setSensorPositions(name, {{"FloatingBase", root_pos}});
      gc->setSensorOrientations(name, {{"FloatingBase", root_ori}});
      gc->setSensorLinearVelocities(name, {{"FloatingBase", root_linvel}});
      gc->setSensorAngularVelocities(name, {{"FloatingBase", root_angvel}});
      gc->setSensorLinearAccelerations(name, {{"FloatingBase", root_linacc}});
      gc->setSensorAngularAccelerations(name, {{"FloatingBase", root_angacc}});
    }
  }

  // Gyro update
  for(auto & gyro : gyros)
  {
    mujoco_get_sensordata(*model, *data, mc_bs_to_mj_gyro_id[gyro.first], gyro.second.data());
    int sid = mc_bs_to_mj_gyro_id[gyro.first];
    if(sid < 0 || model->sensor_objtype[sid] != mjOBJ_SITE)
    {
      continue;
    }
    int site = model->sensor_objid[sid];
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_0_s(data->site_xmat + 9 * site);
    imu_oris[gyro.first] = Eigen::Quaterniond(R_0_s.transpose()); // mc_rtc wants R^T
    imu_poss[gyro.first] = Eigen::Map<const Eigen::Vector3d>(data->site_xpos + 3 * site); // world frame
  }
  gc->setSensorAngularVelocities(name, gyros);
  gc->setSensorOrientations(name, imu_oris);
  gc->setSensorPositions(name, imu_poss);

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
  if(is_external_torques_enabled_changed != use_external_torques)
  {
    // Ensure mc-rtc sees zero external torques when disabled
    tau_ext.setZero();
    robot.setExternalTorques(tau_ext);
    realRobot.setExternalTorques(tau_ext);
    is_external_torques_enabled_changed = use_external_torques;
  }
  if(use_external_torques)
  {
    const auto & mb = robot.mb();
    std::vector<std::vector<double>> tau_ext_mbc(mb.nrJoints());
    for(int j = 0; j < mb.nrJoints(); ++j)
    {
      tau_ext_mbc[j].assign(mb.joint(j).dof(), 0.0);
    }

    // Floating base: MuJoCo free joint qfrc layout is [f (world frame), m (base frame, about base origin)]
    // RBDyn Free joint expects [m; f], both in base frame, about base origin
    // -> reorder, and rotate f from world to base frame
    if(root_qvel_idx != -1 && root_joint_type == mjJNT_FREE && mb.joint(0).dof() == 6)
    {
      // Total external/passive generalized force on the free joint: f in world frame, m in base frame)
      double fx = data->qfrc_constraint[root_qvel_idx + 0] + data->qfrc_passive[root_qvel_idx + 0];
      double fy = data->qfrc_constraint[root_qvel_idx + 1] + data->qfrc_passive[root_qvel_idx + 1];
      double fz = data->qfrc_constraint[root_qvel_idx + 2] + data->qfrc_passive[root_qvel_idx + 2];

      double wx = data->qfrc_constraint[root_qvel_idx + 3] + data->qfrc_passive[root_qvel_idx + 3];
      double wy = data->qfrc_constraint[root_qvel_idx + 4] + data->qfrc_passive[root_qvel_idx + 4];
      double wz = data->qfrc_constraint[root_qvel_idx + 5] + data->qfrc_passive[root_qvel_idx + 5];

      // RBDyn base-frame generalized force [moment; force]
      Eigen::Vector3d f_w(fx, fy, fz);
      const Eigen::Matrix3d R_0_b = root_ori.conjugate().toRotationMatrix(); // undo the inverse
      Eigen::Vector3d f_b = R_0_b.transpose() * f_w; // world -> base
      tau_ext_mbc[0] = {wx, wy, wz, f_b.x(), f_b.y(), f_b.z()};
    }

    // 1-DoF actuated joints
    for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
    {
      int jIndex = mj_to_mbc[i];
      if(jIndex == -1) continue;
      if(mb.joint(jIndex).dof() != 1) continue;
      int dof_addr = model->jnt_dofadr[mj_jnt_ids[i]];
      tau_ext_mbc[jIndex][0] = data->qfrc_constraint[dof_addr] + data->qfrc_passive[dof_addr];
    }

    tau_ext.setZero();
    updateVector(tau_ext_mbc, tau_ext);
    robot.setExternalTorques(tau_ext);
    realRobot.setExternalTorques(tau_ext);
  }
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
      if(torque_control && !mj_is_gripper_joint[i])
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
  if(mj_general_act_id != -1 && mj_general_act_ctrl_idx != -1)
  {
    const auto i = static_cast<size_t>(mj_general_act_ctrl_idx);
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];

    const auto joint_id = mj_jnt_ids[i];
    const double joint_low = model.jnt_range[2 * joint_id];
    const double joint_high = model.jnt_range[2 * joint_id + 1];
    const double ctrl_low = model.actuator_ctrlrange[2 * mj_general_act_id];
    const double ctrl_high = model.actuator_ctrlrange[2 * mj_general_act_id + 1];

    double ctrl = ctrl_low;
    if(joint_high > joint_low)
    {
      const double ratio = (q_ref - joint_low) / (joint_high - joint_low);
      ctrl = ctrl_low + ratio * (ctrl_high - ctrl_low);
    }
    data.ctrl[mj_general_act_id] = std::clamp(ctrl, ctrl_low, ctrl_high);
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
  if(controller->controller().datastore().has("ControlMode"))
  {
    config.torque_control = controller->controller().datastore().get<std::string>("ControlMode").compare("Torque") == 0;
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
  if(!config.with_visualization)
  {
    return;
  }

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
    if(client)
    {
      ImGui::Checkbox("Show mc_rtc visuals", &client->show_visuals);
    }
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
    if(!robots.empty())
    {
      ImGui::Separator();
      ImGui::Text("External torques");
      for(auto & r : robots)
      {
        ImGui::Checkbox(fmt::format("Enable for {}", r.name).c_str(), &r.use_external_torques);
        if(r.use_external_torques)
        {
          ImGui::SameLine();
          ImGui::Text("||tau_ext|| = %.4f", r.tau_ext.norm());
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
  auto user_path = fs::path(USER_FOLDER);
  if(!fs::exists(user_path))
  {
    if(!fs::create_directories(user_path))
    {
      mc_rtc::log::critical("Failed to create the user directory: {}. GUI configuration will not be saved",
                            user_path.string());
      return;
    }
  }

  auto config_path = fmt::format("{}/mc_mujoco.yaml", USER_FOLDER);
  auto config = [&]() -> mc_rtc::Configuration
  {
    if(fs::exists(config_path))
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
