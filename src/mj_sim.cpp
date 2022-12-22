#include "mj_sim_impl.h"
#include "mj_utils.h"

#include <cassert>
#include <chrono>
#include <type_traits>
#include <cstdio>
#include <unordered_map>

#include "MujocoClient.h"
#include "config.h"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot.h"

#include "ImGuizmo.h"

#include "MujocoClient.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <mc_rtc/version.h>

namespace mc_mujoco
{

double MjRobot::PD(double jnt_id, double q_ref, double q, double qdot_ref, double qdot)
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

  int num_joints = joints.size();
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
  auto get_robot_cfg_path = [&](const std::string & robot_name) -> std::string {
    if(bfs::exists(bfs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml")))
    {
      return (bfs::path(mc_mujoco::USER_FOLDER) / (robot_name + ".yaml")).string();
    }
    else if(bfs::exists(bfs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml")))
    {
      return (bfs::path(mc_mujoco::SHARE_FOLDER) / (robot_name + ".yaml")).string();
    }
    else
    {
      return "";
    }
  };

  /** Map between name and xml file path of objects specified in mujoco config **/
  std::map<std::string, std::string> mjObjects;
  /** Map between name and xml file path of objects specified in mc-rtc config **/
  std::map<std::string, std::string> mcObjects;
  /** Map between name and pdgains file path of objects specified in mc-rtc config **/
  std::map<std::string, std::string> pdGainsFiles;

  // load all robots named in mujoco config
  auto mc_mujoco_cfg_path = fmt::format("{}/mc_mujoco.yaml", USER_FOLDER);
  auto mc_mujoco_cfg = [&mc_mujoco_cfg_path]() -> mc_rtc::Configuration {
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
    object.init_pose = static_cast<sva::PTransformd>(co.second("init_pos"));
    objects.push_back(object);

    std::string module = co.second("module");
    auto object_cfg_path = (bfs::path(mc_mujoco::SHARE_FOLDER) / (module + ".yaml")).string();
    if(!bfs::exists(object_cfg_path))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] config cannot be found at {} for {} object",
                                                       object_cfg_path, co.first);
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
#if MC_RTC_VERSION_MAJOR > 1
  for(const auto & r_ptr : controller->robots())
  {
    const auto & r = *r_ptr;
#else
  for(const auto & r : controller->robots())
  {
#endif
    const auto & robot_cfg_path = get_robot_cfg_path(r.module().name);
    if(robot_cfg_path.size())
    {
      auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);
      if(!robot_cfg.has("xmlModelPath"))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath in {}", robot_cfg_path);
      }
      std::string xmlFile = static_cast<std::string>(robot_cfg("xmlModelPath"));
      mcObjects[r.name()] = xmlFile;
      pdGainsFiles[r.name()] = robot_cfg("pdGainsPath", std::string(""));
      if(!bfs::exists(xmlFile))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {} for {}", xmlFile,
                                                         r.name());
      }
    }
  }

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
        std::any_of(r.mj_mot_names.begin(), r.mj_mot_names.end(), [](const std::string & m) { return m.size() != 0; });
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

void MjRobot::initialize(mjModel * model, const mc_rbdyn::Robot & robot)
{
  mj_jnt_ids.resize(0);
  for(const auto & j : mj_jnt_names)
  {
    mj_jnt_ids.push_back(mj_name2id(model, mjOBJ_JOINT, j.c_str()));
  }
  auto fill_acuator_ids = [&](const std::vector<std::string> & names, std::vector<int> & ids) {
    ids.resize(0);
    for(const auto & n : names)
    {
      if(n.size())
      {
        ids.push_back(mj_name2id(model, mjOBJ_ACTUATOR, n.c_str()));
      }
      else
      {
        ids.push_back(-1);
      }
    }
  };
  fill_acuator_ids(mj_mot_names, mj_mot_ids);
  fill_acuator_ids(mj_pos_act_names, mj_pos_act_ids);
  fill_acuator_ids(mj_vel_act_names, mj_vel_act_ids);
  if(root_body.size())
  {
    root_body_id = mj_name2id(model, mjOBJ_BODY, root_body.c_str());
  }
  auto init_sensor_id = [&](const char * mj_name, const char * mc_name, const std::string & sensor_name,
                            const char * suffix, mjtSensor type, std::unordered_map<std::string, int> & mapping) {
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
  mj_jnt_to_rjo.resize(0);
  mj_to_mbc.resize(0);
  encoders = std::vector<double>(rjo.size(), 0.0);
  alphas = std::vector<double>(rjo.size(), 0.0);
  torques = std::vector<double>(rjo.size(), 0.0);
  for(const auto & mj_jn : mj_jnt_names)
  {
    const auto & jn = [&]() {
      if(prefix.size())
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
      if(rjo_idx != -1)
      {
        encoders[rjo_idx] = mj_prev_ctrl_q.back();
        alphas[rjo_idx] = mj_prev_ctrl_alpha.back();
      }
    }
    else
    {
      mj_to_mbc.push_back(-1);
    }
  }
  mj_ctrl = mj_prev_ctrl_q;
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;

  // reset the PD gains to default values
  kp = default_kp;
  kd = default_kd;
}

void MjSimImpl::setSimulationInitialState()
{
  if(controller)
  {
    qInit.resize(0);
    alphaInit.resize(0);

    for(auto & o : objects)
    {
      sva::PTransformd pose = o.init_pose;
      const auto & t = pose.translation();
      for(size_t i = 0; i < 3; ++i)
      {
        qInit.push_back(t[i]);
        alphaInit.push_back(0);
        alphaInit.push_back(0);
      }
      Eigen::Quaterniond q = Eigen::Quaterniond(pose.rotation()).inverse();
      qInit.push_back(q.w());
      qInit.push_back(q.x());
      qInit.push_back(q.y());
      qInit.push_back(q.z());
    }

    for(auto & r : robots)
    {
      const auto & robot = controller->robots().robot(r.name);
      r.initialize(model, robot);
      if(r.root_joint.size())
      {
        r.root_qpos_idx = qInit.size();
        r.root_qvel_idx = alphaInit.size();
        if(robot.mb().joint(0).dof() == 6)
        {
          const auto & t = robot.posW().translation();
          for(size_t i = 0; i < 3; ++i)
          {
            qInit.push_back(t[i]);
            // push linear/angular velocities
            alphaInit.push_back(0);
            alphaInit.push_back(0);
          }
          Eigen::Quaterniond q = Eigen::Quaterniond(robot.posW().rotation()).inverse();
          qInit.push_back(q.w());
          qInit.push_back(q.x());
          qInit.push_back(q.y());
          qInit.push_back(q.z());
        }
      }
      else if(r.root_body_id != -1)
      {
        const auto & t = robot.posW().translation();
        model->body_pos[3 * r.root_body_id + 0] = t.x();
        model->body_pos[3 * r.root_body_id + 1] = t.y();
        model->body_pos[3 * r.root_body_id + 2] = t.z();
        Eigen::Quaterniond q = Eigen::Quaterniond(robot.posW().rotation()).inverse();
        model->body_quat[4 * r.root_body_id + 0] = q.w();
        model->body_quat[4 * r.root_body_id + 1] = q.x();
        model->body_quat[4 * r.root_body_id + 2] = q.y();
        model->body_quat[4 * r.root_body_id + 3] = q.z();
      }
      for(size_t i = 0; i < r.mj_jnt_names.size(); ++i)
      {
        qInit.push_back(r.encoders[r.mj_jnt_to_rjo[i]]);
        alphaInit.push_back(r.alphas[r.mj_jnt_to_rjo[i]]);
      }
    }
  }
  // set initial qpos, qvel in mujoco
  if(!mujoco_set_const(model, data, qInit, alphaInit))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Set inital state failed.");
  }
  mj_forward(model, data);
}

void MjSimImpl::makeDatastoreCalls()
{

  // store the name, position, and orientation of the object from the simulation into the DataStore
  std::vector<std::string> objectsNames;
  std::vector<double> objectsPositions;
  std::vector<double> objectsOrientations;
  std::vector<double> handsPositions;
  std::vector<double> handsOrientations;
  std::unordered_map<std::string, int> mapOfObjectsNames;
  listOfHandIndex.resize(0);

  for (unsigned int i = 0; i < model->nbody; i++){

    std::string objectName = mj_id2name(model, mjOBJ_BODY, i);
    if (i > 0 && i < 10 && objectName.compare("longtable_base_link") != 0)
    {
      objectName = objectName.substr(0, objectName.find('_'));
      objectsNames.push_back(objectName);
      objectsPositions.push_back(data->xpos[i*3]);
      objectsPositions.push_back(data->xpos[i*3+1]);
      objectsPositions.push_back(data->xpos[i*3+2]);
      objectsOrientations.push_back(data->xquat[i*4]);
      objectsOrientations.push_back(data->xquat[i*4+1]);
      objectsOrientations.push_back(data->xquat[i*4+2]);
      objectsOrientations.push_back(data->xquat[i*4+3]);
      listOfObjectIndex.push_back(i);
      mapOfObjectsNames[objectName] = i;
      std::cout << "Name: "<< objectName << ", id: " << i;
      std::cout << ", position: (" << data->xpos[i*3] << ", " << data->xpos[i*3+1] << ", " << data->xpos[i*3+2] << ")";
      std::cout << " and orientation: (" << data->xquat[i*4] << ", " << data->xquat[i*4+1] << ", " << data->xquat[i*4+2] << ", " << data->xquat[i*4+3] <<")\n";
    }

    else if(objectName.compare("hrp4cr_R_HAND_MIDDLE_J0_LINK") == 0 || objectName.compare("hrp4cr_L_HAND_MIDDLE_J0_LINK") == 0)
    {
      std::cout << "Add a hand to the dataStore" << std::endl;
      handsPositions.push_back(data->xpos[i*3]);
      handsPositions.push_back(data->xpos[i*3+1]);
      handsPositions.push_back(data->xpos[i*3+2]);
      handsOrientations.push_back(data->xquat[i*4]);
      handsOrientations.push_back(data->xquat[i*4+1]);
      handsOrientations.push_back(data->xquat[i*4+2]);
      handsOrientations.push_back(data->xquat[i*4+3]);
      listOfHandIndex.push_back(i);
      std::cout << "Name: "<< objectName << ", id: " << i;
      std::cout << ", position: (" << data->xpos[i*3] << ", " << data->xpos[i*3+1] << ", " << data->xpos[i*3+2] << ")";
      std::cout << " and orientation: (" << data->xquat[i*4] << ", " << data->xquat[i*4+1] << ", " << data->xquat[i*4+2] << ", " << data->xquat[i*4+3] <<")\n";
    }
  }
  std::cout << std::endl;

  std::unordered_map<int, std::vector<double>> mapOfGoalPositions;
  std::unordered_map<int, std::vector<double>> mapOfGoalOrientations;

  std::cout << "List of geoms to grab in the sim:" << std::endl;
  for (unsigned int i = 0; i < model->ngeom; i++)
  {
    const char* geomNameTmp = mj_id2name(model, mjOBJ_GEOM, i);
    if(geomNameTmp != NULL)
    {
      std::cout << "Name: "<< geomNameTmp << ", id: " << i;
      Eigen::Matrix3d rotationMatrix;
      rotationMatrix << data->geom_xmat[i*9], data->geom_xmat[i*9+1], data->geom_xmat[i*9+2], data->geom_xmat[i*9+3], data->geom_xmat[i*9+4],
                        data->geom_xmat[i*9+5], data->geom_xmat[i*9+6], data->geom_xmat[i*9+7], data->geom_xmat[i*9+8]; 
      Eigen::Quaterniond geomQuat(rotationMatrix);
      std::cout << ", position: (" << data->geom_xpos[i*3] << ", " << data->geom_xpos[i*3+1] << ", " << data->geom_xpos[i*3+2] << ")";
      std::cout << " and orientation: (" << geomQuat.w() << ", " << geomQuat.x() << ", " << geomQuat.y() << ", " << geomQuat.z() <<")\n";
      std::string geomName = geomNameTmp;
      geomName = geomName.substr(0, geomName.find('_'));
      int objectIndex = mapOfObjectsNames[geomName];
      mapOfGoalPositions[objectIndex].push_back(data->geom_xpos[i*3]);
      mapOfGoalPositions[objectIndex].push_back(data->geom_xpos[i*3+1]);
      mapOfGoalPositions[objectIndex].push_back(data->geom_xpos[i*3+2]);
      mapOfGoalOrientations[objectIndex].push_back(geomQuat.w());
      mapOfGoalOrientations[objectIndex].push_back(geomQuat.x());
      mapOfGoalOrientations[objectIndex].push_back(geomQuat.y());
      mapOfGoalOrientations[objectIndex].push_back(geomQuat.z());
      mapOfGeomIndex[objectIndex].push_back(i);
    }
  }

  controller->controller().datastore().make<std::vector<std::string>>("objectsNames", objectsNames);
  controller->controller().datastore().make<std::vector<double>>("objectsPositions", objectsPositions);
  controller->controller().datastore().make<std::vector<double>>("objectsOrientations", objectsOrientations);
  controller->controller().datastore().make<std::vector<double>>("handsPositions", handsPositions);
  controller->controller().datastore().make<std::vector<double>>("handsOrientations", handsOrientations);
  controller->controller().datastore().make<std::unordered_map<int, std::vector<double>>>("mapOfGoalPositions", mapOfGoalPositions);
  controller->controller().datastore().make<std::unordered_map<int, std::vector<double>>>("mapOfGoalOrientations", mapOfGoalOrientations);
  mc_rtc::log::info("Reaches Very first step");

  for(auto & r : robots)
  {
    // make_call for setting pd gains (for all joints)
    controller->controller().datastore().make_call(
        r.name + "::SetPDGains", [this, &r](const std::vector<double> & p_vec, const std::vector<double> & d_vec) {
          const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
          if(p_vec.size() != rjo.size())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. p_vec size({})!=ref_joint_order size({})", r.name,
                                 p_vec.size(), rjo.size());
            return false;
          }
          if(d_vec.size() != rjo.size())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::SetPDGains failed. d_vec size({})!=ref_joint_order size({})", r.name,
                                 d_vec.size(), rjo.size());
            return false;
          }
          r.kp = p_vec;
          r.kd = d_vec;
          return true;
        });

    // make_call for setting pd gains (by name)
    controller->controller().datastore().make_call(
        r.name + "::SetPDGainsByName", [this, &r](const std::string & jn, double p, double d) {
          const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
          auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
          if(rjo_it == rjo.end())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::SetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                                 r.name, jn);
            return false;
          }
          int rjo_idx = std::distance(rjo.begin(), rjo_it);
          r.kp[rjo_idx] = p;
          r.kd[rjo_idx] = d;
          return true;
        });

    // make_call for reading pd gains (for all joints)
    controller->controller().datastore().make_call(
        r.name + "::GetPDGains", [this, &r](std::vector<double> & p_vec, std::vector<double> & d_vec) {
          p_vec.resize(0);
          d_vec.resize(0);
          p_vec = r.kp;
          d_vec = r.kd;
          const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
          if(p_vec.size() != rjo.size())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. p_vec size({})!=ref_joint_order size({})", r.name,
                                 p_vec.size(), rjo.size());
            return false;
          }
          if(d_vec.size() != rjo.size())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::GetPDGains failed. d_vec size({})!=ref_joint_order size({})", r.name,
                                 d_vec.size(), rjo.size());
            return false;
          }
          return true;
        });

    // make_call for reading pd gains (by name)
    controller->controller().datastore().make_call(
        r.name + "::GetPDGainsByName", [this, &r](const std::string & jn, double & p, double & d) {
          const auto & rjo = controller->robots().robot(r.name).module().ref_joint_order();
          auto rjo_it = std::find(rjo.begin(), rjo.end(), jn);
          if(rjo_it == rjo.end())
          {
            mc_rtc::log::warning("[mc_mujoco] {}::GetPDGainsByName failed. Joint {} not found in ref_joint_order.",
                                 r.name, jn);
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
  setSimulationInitialState();
  if(!config.with_controller)
  {
    controller.reset();
    return;
  }

  makeDatastoreCalls();
  timer = std::chrono::system_clock::now();

  // get sim timestep and set the frameskip parameter
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  for(const auto & r : robots)
  {
    controller->setEncoderValues(r.name, r.encoders);
  }
  for(const auto & r : robots)
  {
    init_qs_[r.name] = r.encoders;
    init_pos_[r.name] = controller->controller().robot(r.name).posW();
  }
  controller->init(init_qs_, init_pos_);
  controller->running = true;
}

void MjSimImpl::updateTeleopData(){
  auto & objectsPositions = controller->controller().datastore().get<std::vector<double>>("objectsPositions");
  auto & objectsOrientations = controller->controller().datastore().get<std::vector<double>>("objectsOrientations");
  auto & handsPositions = controller->controller().datastore().get<std::vector<double>>("handsPositions");
  auto & handsOrientations = controller->controller().datastore().get<std::vector<double>>("handsOrientations");
  auto & mapOfGoalPositions = controller->controller().datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalPositions");
  auto & mapOfGoalOrientations = controller->controller().datastore().get<std::unordered_map<int, std::vector<double>>>("mapOfGoalOrientations");
  for (unsigned int i = 0; i < listOfObjectIndex.size(); i++){
    int index = listOfObjectIndex[i]*3;
    objectsPositions[i*3] = data->xpos[index];
    objectsPositions[i*3+1] = data->xpos[index+1];
    objectsPositions[i*3+2] = data->xpos[index+2];
    index = listOfObjectIndex[i]*4;
    objectsOrientations[i*4] = data->xquat[index];
    objectsOrientations[i*4+1] = data->xquat[index+1];
    objectsOrientations[i*4+2] = data->xquat[index+2];
    objectsOrientations[i*4+3] = data->xquat[index+3];
    int j = 0;
    while(j < mapOfGoalPositions[i].size()/3)
    {
      int geomIndex = mapOfGeomIndex[i][j];
      mapOfGoalPositions[i][j*3] = data->geom_xpos[geomIndex];
      mapOfGoalPositions[i][j*3+1] = data->geom_xpos[geomIndex+1];
      mapOfGoalPositions[i][j*3+2] = data->geom_xpos[geomIndex+2];
      Eigen::Matrix3d rotationMatrix;
      rotationMatrix << data->geom_xmat[geomIndex*9], data->geom_xmat[geomIndex*9+1], data->geom_xmat[geomIndex*9+2], data->geom_xmat[geomIndex*9+3], data->geom_xmat[geomIndex*9+4],
                        data->geom_xmat[geomIndex*9+5], data->geom_xmat[geomIndex*9+6], data->geom_xmat[geomIndex*9+7], data->geom_xmat[geomIndex*9+8]; 
      Eigen::Quaterniond geomQuat(rotationMatrix);
      mapOfGoalOrientations[i][j*4] = geomQuat.w();
      mapOfGoalOrientations[i][j*4+1] = geomQuat.x();
      mapOfGoalOrientations[i][j*4+2] = geomQuat.y();
      mapOfGoalOrientations[i][j*4+3] = geomQuat.z();
      j++; 
    }
  }
  for (unsigned int i = 0; i < listOfHandIndex.size(); i++){
    int index = listOfHandIndex[i]*3;
    handsPositions[i*3] = data->xpos[index];
    handsPositions[i*3+1] = data->xpos[index+1];
    handsPositions[i*3+2] = data->xpos[index+2];
    index = listOfHandIndex[i]*4;
    handsOrientations[i*4] = data->xquat[index];
    handsOrientations[i*4+1] = data->xquat[index+1];
    handsOrientations[i*4+2] = data->xquat[index+2];
    handsOrientations[i*4+3] = data->xquat[index+3];
  }
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
  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - timer;
  if (elapsed_time.count() > 0.250)
  {
    updateTeleopData();
    timer = std::chrono::system_clock::now();
  }
}

void MjRobot::updateControl(const mc_rbdyn::Robot & robot)
{
  mj_prev_ctrl_q = mj_next_ctrl_q;
  mj_prev_ctrl_alpha = mj_next_ctrl_alpha;
  size_t ctrl_idx = 0;
  for(size_t i = 0; i < mj_to_mbc.size(); ++i)
  {
    auto jIndex = mj_to_mbc[i];
    if(jIndex != -1)
    {
      mj_next_ctrl_q[ctrl_idx] = robot.mbc().q[jIndex][0];
      mj_next_ctrl_alpha[ctrl_idx] = robot.mbc().alpha[jIndex][0];
      ctrl_idx++;
    }
  }
}

void MjRobot::sendControl(const mjModel & model, mjData & data, size_t interp_idx, size_t frameskip_)
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
    if(mot_id != -1)
    {
      // compute desired torque using PD control
      mj_ctrl[i] = PD(i, q_ref, encoders[rjo_id], alpha_ref, alphas[rjo_id]);
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
    r.sendControl(*model, *data, interp_idx, frameskip_);
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
  auto do_step = [this, &start_step]() {
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
  mjr_render(uistate.rect[0], &scene, &context);

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
    client->draw2D(window);
    //client->draw3D();
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
      auto doNStepsButton = [&](size_t n, bool final_) {
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
    auto flag_to_gui = [&](const char * label, mjtVisFlag flag) {
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
    auto group_to_checkbox = [&](size_t group, bool last) {
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
    ImGui::End();
  }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  return !glfwWindowShouldClose(window);
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
  auto config = [&]() -> mc_rtc::Configuration {
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
  config.save(config_path);
  mc_rtc::log::success("[mc_mujoco] Configuration saved to {}", config_path);
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

} // namespace mc_mujoco
