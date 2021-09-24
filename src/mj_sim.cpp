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

#include "MujocoClient.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

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

  std::vector<std::string> mujRobots;
  std::vector<std::string> xmlFiles;
  std::vector<std::string> pdGainsFiles;
  for(const auto & r : controller->robots())
  {
    const auto & robot_cfg_path = get_robot_cfg_path(r.module().name);
    if(robot_cfg_path.size())
    {
      auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);
      if(!robot_cfg.has("xmlModelPath") || !robot_cfg.has("pdGainsPath"))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath or pdGainsPath in {}", robot_cfg_path);
      }
      mujRobots.push_back(r.name());
      xmlFiles.push_back(static_cast<std::string>(robot_cfg("xmlModelPath")));
      pdGainsFiles.push_back(static_cast<std::string>(robot_cfg("pdGainsPath")));
      if(!bfs::exists(xmlFiles.back()))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {}",
                                                         xmlFiles.back());
      }
      if(!bfs::exists(pdGainsFiles.back()))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains file cannot be found at {}",
                                                         pdGainsFiles.back());
      }
    }
  }

  if(!xmlFiles.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No Mujoco model associated to any robots in the controller");
  }

  // initial mujoco here and load XML model
  bool initialized = mujoco_init(this, mujRobots, xmlFiles);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }

  // read PD gains from file
  for(size_t i = 0; i < robots.size(); ++i)
  {
    auto & r = robots[i];
    r.loadGain(pdGainsFiles[i], controller->robots().robot(r.name).module().ref_joint_order());
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
  for(const auto & j : mj_jnt_names)
  {
    mj_jnt_ids.push_back(mj_name2id(model, mjOBJ_JOINT, j.c_str()));
  }
  for(const auto & m : mj_mot_names)
  {
    mj_mot_ids.push_back(mj_name2id(model, mjOBJ_ACTUATOR, m.c_str()));
  }
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
  for(const auto & mj_jn : mj_jnt_names)
  {
    const auto & jn = [&]() {
      if(prefix.size())
      {
        return mj_jn.substr(prefix.size() + 1);
      }
      return mj_jn;
    }();
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
      encoders.push_back(mj_prev_ctrl_q.back());
      alphas.push_back(mj_prev_ctrl_alpha.back());
    }
    else
    {
      mj_to_mbc.push_back(-1);
      encoders.push_back(0);
      alphas.push_back(0);
    }
    torques.push_back(0);
  }
  mj_ctrl = mj_prev_ctrl_q;
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;
  for(const auto & fs : robot.module().forceSensors())
  {
    wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
  }
}

void MjSimImpl::startSimulation()
{
  std::vector<double> qInit;
  std::vector<double> alphaInit;
  for(auto & r : robots)
  {
    const auto & robot = controller->robots().robot(r.name);
    r.initialize(model, robot);
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
      for(const auto & q : r.encoders)
      {
        qInit.push_back(q);
      }
      for(const auto & a : r.alphas)
      {
        alphaInit.push_back(a);
      }
    }
  }
  // set initial qpos, qvel in mujoco
  if(!mujoco_set_const(model, data, qInit, alphaInit))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Set inital state failed.");
  }

  if(!config.with_controller)
  {
    controller.reset();
    return;
  }

  // get sim timestep and set the frameskip parameter
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  for(const auto & r : robots)
  {
    controller->setEncoderValues(r.name, r.encoders);
  }
  controller->init(robots[0].encoders);
  controller->running = true;
}

void MjRobot::updateSensors(mc_control::MCGlobalController * gc, mjModel * model, mjData * data)
{
  for(size_t i = 0; i < mj_jnt_ids.size(); ++i)
  {
    encoders[i] = data->qpos[model->jnt_qposadr[mj_jnt_ids[i]]];
    alphas[i] = data->qvel[model->jnt_dofadr[mj_jnt_ids[i]]];
  }
  for(size_t i = 0; i < mj_mot_ids.size(); ++i)
  {
    torques[i] = data->qfrc_actuator[mj_mot_ids[i]];
  }
  if(!gc)
  {
    return;
  }
  auto & robot = gc->controller().robots().robot(name);

  // Returns the prefixed name of a sensor
  auto prefixed = [&](const std::string & name) {
    if(prefix.size())
    {
      return fmt::format("{}_{}", prefix, name);
    }
    else
    {
      return name;
    }
  };

  // Body sensor updates
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
  gc->setSensorPosition(name, root_pos);
  gc->setSensorOrientation(name, root_ori);
  gc->setSensorLinearVelocity(name, root_linvel);
  gc->setSensorAngularVelocity(name, root_angvel);
  gc->setSensorLinearAcceleration(name, root_linacc);
  // FIXME Not implemented in mc_rtc
  // gc->setSensorAngularAcceleration(name, root_linacc);
  // set angular velocity
  std::vector<double> _gyro;
  mujoco_get_sensordata(*model, *data, _gyro, prefixed("root_gyro"));
  if(_gyro.size() != 3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] gyro read failed.");
  gc->setSensorAngularVelocities(name, {{"Accelerometer", Eigen::Vector3d(_gyro.data())}});

  // set linear acceleration
  std::vector<double> _accel;
  mujoco_get_sensordata(*model, *data, _accel, prefixed("root_accel"));
  if(_accel.size() != 3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] accel read failed.");
  gc->setSensorLinearAccelerations(name, {{"Accelerometer", Eigen::Vector3d(_accel.data())}});

  // Joint sensor updates
  gc->setEncoderValues(name, encoders);
  gc->setEncoderVelocities(name, alphas);
  gc->setJointTorques(name, torques);

  // Force sensors
  std::vector<double> _fsensor, _tsensor;
  auto getWrench = [&](const std::string & mc_sensor, const std::string & mj_fsensor, const std::string & mj_tsensor) {
    if(mujoco_get_sensordata(*model, *data, _fsensor, prefixed(mj_fsensor))
       && mujoco_get_sensordata(*model, *data, _tsensor, prefixed(mj_tsensor)))
    {
      auto & w = wrenches[mc_sensor];
      w.force() = -1 * Eigen::Map<Eigen::Vector3d>(_fsensor.data());
      w.couple() = -1 * Eigen::Map<Eigen::Vector3d>(_tsensor.data());
    }
  };
  getWrench("RightFootForceSensor", "rf_fsensor", "rf_tsensor");
  getWrench("LeftFootForceSensor", "lf_fsensor", "lf_tsensor");
  getWrench("RightHandForceSensor", "rh_fsensor", "rh_tsensor");
  getWrench("LeftHandForceSensor", "lh_fsensor", "lh_tsensor");
  gc->setWrenches(wrenches);
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
    // compute desired q using interpolation
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];
    // compute desired alpha using interpolation
    double alpha_ref = (interp_idx + 1) * (mj_next_ctrl_alpha[i] - mj_prev_ctrl_alpha[i]) / frameskip_;
    alpha_ref += mj_prev_ctrl_alpha[i];
    // compute desired torque using PD control
    mj_ctrl[i] = PD(i, q_ref, encoders[i], alpha_ref, alphas[i]);
    double ratio = model.actuator_gear[6 * mj_mot_ids[i]];
    data.ctrl[mj_mot_ids[i]] = mj_ctrl[i] / ratio;
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
  // take one step in simulation
  // model.opt.timestep will be used here
  mj_step(model, data);
}

bool MjSimImpl::stepSimulation()
{
  auto start_step = clock::now();
  // Only run the GUI update if the simulation is paused
  if(config.step_by_step && rem_steps == 0 && controller)
  {
    controller->running = false;
    controller->run();
    controller->running = true;
    mj_sim_start_t = start_step;
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
    simStep();
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
    done = do_step();
    rem_steps--;
  }
  if(config.sync_real_time)
  {
    std::this_thread::sleep_until(start_step + duration_us(1e6 * model->opt.timestep) + mj_sync_delay);
  }
  return done;
}

bool MjSimImpl::render()
{
  if(!config.with_visualization)
  {
    return true;
  }
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(model, data, &options, NULL, &camera, mjCAT_ALL, &scene);

  if(client)
  {
    client->updateScene(scene);
  }

  mjr_render(viewport, &scene, &context);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();

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
    client->draw3D();
  }
  {
    auto right_margin = 5.0f;
    auto top_margin = 5.0f;
    auto width = io.DisplaySize.x - 2 * right_margin;
    auto height = io.DisplaySize.y - 2 * top_margin;
    ImGui::SetNextWindowPos({0.8f * width - right_margin, top_margin}, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({0.2f * width, 0.2f * height}, ImGuiCond_FirstUseEver);
    ImGui::Begin("mc_mujoco");
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
    ImGui::End();
  }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  return !glfwWindowShouldClose(window);
}

void MjSimImpl::stopSimulation() {}

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

bool MjSim::render()
{
  return impl->render();
}

} // namespace mc_mujoco
