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

double MjSimImpl::PD(double jnt_id, double q_ref, double q, double qdot_ref, double qdot)
{
  double p_error = q_ref - q;
  double v_error = qdot_ref - qdot;
  double ret = (kp[jnt_id] * p_error + kd[jnt_id] * v_error);
  return ret;
}

/* Load PD gains from file (taken from RobotHardware/robot.cpp) */
bool MjSimImpl::loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints)
{
  std::ifstream strm(path_to_pd.c_str());
  if(!strm.is_open())
  {
    std::cerr << path_to_pd << " not found" << std::endl;
    return false;
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
        std::cerr << "[mc_mujoco] loadGain error: size of gains reading from file (" << path_to_pd
                  << ") does not match size of joints" << std::endl;
      }
      break;
    }
  }

  strm.close();
  // Print loaded gain
  std::cerr << "[mc_mujoco] loadGain" << std::endl;
  for(unsigned int i = 0; i < num_joints; i++)
  {
    std::cerr << "[mc_mujoco]   " << joints[i] << ", pgain = " << default_pgain[i] << ", dgain = " << default_dgain[i]
              << std::endl;
    // push to kp and kd
    kp.push_back(default_pgain[i]);
    kd.push_back(default_dgain[i]);
  }
  return true;
}

MjSimImpl::MjSimImpl(const MjConfiguration & config)
: controller(std::make_unique<mc_control::MCGlobalController>(config.mc_config)), config(config)
{
  const auto & robot_name = controller->robot().module().name;
  auto get_robot_cfg_path = [&]() -> std::string {
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

  std::string xmlPath = "";
  std::string pdGains = "";
  const auto & robot_cfg_path = get_robot_cfg_path();
  if(robot_cfg_path.size())
  {
    auto robot_cfg = mc_rtc::Configuration(robot_cfg_path);
    if(!robot_cfg.has("xmlModelPath") || !robot_cfg.has("pdGainsPath"))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Missing xmlModelPath or pdGainsPath in {}", robot_cfg_path);
    }
    xmlPath = static_cast<std::string>(robot_cfg("xmlModelPath"));
    pdGains = static_cast<std::string>(robot_cfg("pdGainsPath"));
  }
  else
  {
    auto mj_c = controller->configuration().config("MUJOCO", mc_rtc::Configuration{});
    mj_c("xmlModelPath", xmlPath);
    mj_c("pdGainsPath", pdGains);
  }

  if(!bfs::exists(xmlPath))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] XML model cannot be found at {}", xmlPath);
  }
  if(!bfs::exists(pdGains))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains file cannot be found at {}", pdGains);
  }

  // initial mujoco here and load XML model
  bool initialized = mujoco_init(this, xmlPath.c_str(), config.with_visualization);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }

  // read PD gains from file
  const std::vector<std::string> rjo = controller->ref_joint_order();
  if(!loadGain(pdGains, rjo))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] PD gains load failed.");
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

void MjSimImpl::startSimulation()
{
  // get names and ids of all joints
  mujoco_get_joints(*model, mj_jnt_names, mj_jnt_ids);
  // get names and ids of actuated joints
  mujoco_get_motors(*model, mj_mot_names, mj_mot_ids);

  mj_to_mbc.resize(0);
  mj_prev_ctrl_q.resize(0);
  mj_prev_ctrl_alpha.resize(0);
  const auto & robot = controller->robot();
  for(const auto & jn : mj_mot_names)
  {
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
    }
    else
    {
      mj_to_mbc.push_back(-1);
    }
  }
  mj_ctrl = mj_prev_ctrl_q;
  mj_next_ctrl_q = mj_prev_ctrl_q;
  mj_next_ctrl_alpha = mj_prev_ctrl_alpha;

  // init attitude from mc-rtc
  const auto q0 = robot.module().default_attitude();
  std::vector<double> mj_qpos_init{q0[4], q0[5], q0[6], q0[0], q0[1], q0[2], q0[3]};
  std::vector<double> mj_qvel_init{0, 0, 0, 0, 0, 0};

  // init qpos and qvel from mc-rtc
  {
    auto & mbc = robot.mbc();
    const auto & rjo = controller->ref_joint_order();
    for(const auto & jn : rjo)
    {
      if(robot.hasJoint(jn))
      {
        for(auto & qj : mbc.q[robot.jointIndexByName(jn)])
        {
          encoders.push_back(qj);
          if(std::find(mj_jnt_names.begin(), mj_jnt_names.end(), jn) != mj_jnt_names.end())
          {
            mj_qpos_init.push_back(qj);
            mj_qvel_init.push_back(0);
          }
        }
      }
      else
      {
        // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
        // likely to be true)
        encoders.push_back(0);
        mj_qpos_init.push_back(0);
        mj_qvel_init.push_back(0);
      }
    }
  }
  alphas.resize(encoders.size());
  std::fill(alphas.begin(), alphas.end(), 0);

  // sanity check
  if(encoders.size() != mj_jnt_names.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Num encoders size mismatch.");
  }
  // zero force/torque wrenches
  for(const auto & fs : robot.module().forceSensors())
  {
    wrenches[fs.name()] = sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
  }

  // set initial qpos, qvel in mujoco
  if(!mujoco_set_const(model, data, mj_qpos_init, mj_qvel_init))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Set inital state failed.");
  }

  // get sim timestep and set the frameskip parameter
  double simTimestep = model->opt.timestep;
  frameskip_ = std::round(controller->timestep() / simTimestep);
  mc_rtc::log::info("[mc_mujoco] MC-RTC timestep: {}. MJ timestep: {}", controller->timestep(), simTimestep);
  mc_rtc::log::info("[mc_mujoco] Hence, Frameskip: {}", frameskip_);

  controller->setEncoderValues(encoders);
  controller->init(encoders);
  controller->running = true;
}

void MjSimImpl::updateData()
{
  auto & robot = controller->controller().robots().robot();

  /* True state of floating base */
  if(robot.hasBodySensor("FloatingBase"))
  {
    // set root position
    mujoco_get_root_pos(*data, root_pos);
    controller->setSensorPositions({{"FloatingBase", root_pos}});
    // set root orientation
    mujoco_get_root_orient(*data, root_orient);
    controller->setSensorOrientations({{"FloatingBase", root_orient}});
    // set linear velocity
    mujoco_get_root_lin_vel(*data, root_linvel);
    controller->setSensorLinearVelocities({{"FloatingBase", root_linvel}});
    // set angular velocity
    mujoco_get_root_ang_vel(*data, root_angvel);
    controller->setSensorAngularVelocities({{"FloatingBase", root_angvel}});
    // set linear acceleration
    mujoco_get_root_lin_acc(*data, root_linacc);
    controller->setSensorLinearAccelerations({{"FloatingBase", root_linacc}});
    // set angular acceleration
    mujoco_get_root_ang_acc(*data, root_angacc);
    controller->setSensorAngularAccelerations({{"FloatingBase", root_angacc}});
  }
  controller->setSensorPosition(root_pos);
  controller->setSensorOrientation(root_orient);
  controller->setSensorLinearVelocity(root_linvel);
  controller->setSensorAngularVelocity(root_angvel);
  controller->setSensorLinearAcceleration(root_linacc);
  controller->setSensorAngularAcceleration(root_linacc);

  /* State from sensor data*/
  // set angular velocity
  std::vector<double> _gyro;
  mujoco_get_sensordata(*model, *data, _gyro, "root_gyro");
  if(_gyro.size() != 3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] gyro read failed.");
  controller->setSensorAngularVelocities({{"Accelerometer", Eigen::Vector3d(_gyro.data())}});

  // set linear acceleration
  std::vector<double> _accel;
  mujoco_get_sensordata(*model, *data, _accel, "root_accel");
  if(_accel.size() != 3) mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] accel read failed.");
  controller->setSensorLinearAccelerations({{"Accelerometer", Eigen::Vector3d(_accel.data())}});

  // set encoders
  encoders.resize(robot.refJointOrder().size());
  mujoco_get_joint_pos(*model, *data, encoders);
  controller->setEncoderValues(encoders);

  // set velocities
  alphas.resize(robot.refJointOrder().size());
  mujoco_get_joint_vel(*model, *data, alphas);
  controller->setEncoderVelocities(alphas);

  // set joint torques
  std::vector<double> qfrc;
  mujoco_get_joint_qfrc(*model, *data, qfrc);
  controller->setJointTorques(qfrc);

  // Read force sensors
  std::vector<double> _fsensor, _tsensor;
  auto getWrench = [&](const std::string & mc_sensor, const std::string & mj_fsensor, const std::string & mj_tsensor) {
    if(mujoco_get_sensordata(*model, *data, _fsensor, mj_fsensor)
       && mujoco_get_sensordata(*model, *data, _tsensor, mj_tsensor))
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
  controller->setWrenches(wrenches);
}

bool MjSimImpl::controlStep()
{
  auto interp_idx = iterCount_ % frameskip_;
  // After every frameskip iters
  if(interp_idx == 0)
  {
    // run the controller
    if(!controller->run())
    {
      return true;
    }
    // and get QP result
    mj_prev_ctrl_q = mj_next_ctrl_q;
    mj_prev_ctrl_alpha = mj_next_ctrl_alpha;
    const auto & robot = controller->robot();
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
  // On each control iter
  for(size_t i = 0; i < mj_ctrl.size(); ++i)
  {
    auto jnt_idx = mj_mot_ids[i] - 1; // subtract 1 because mujoco counts from the "freejoint"
    // compute desired q using interpolation
    double q_ref = (interp_idx + 1) * (mj_next_ctrl_q[i] - mj_prev_ctrl_q[i]) / frameskip_;
    q_ref += mj_prev_ctrl_q[i];
    // compute desired alpha using interpolation
    double alpha_ref = (interp_idx + 1) * (mj_next_ctrl_alpha[i] - mj_prev_ctrl_alpha[i]) / frameskip_;
    alpha_ref += mj_prev_ctrl_alpha[i];
    // compute desired torque using PD control
    mj_ctrl[i] = PD(jnt_idx, q_ref, encoders[jnt_idx], alpha_ref, alphas[jnt_idx]);
  }
  iterCount_++;
  // send control signal to mujoco
  return !(controller->running && mujoco_set_ctrl(*model, *data, mj_ctrl));
}

void MjSimImpl::simStep()
{
  // take one step in simulation
  // model.opt.timestep will be used here
  mj_step(model, data);
}

bool MjSimImpl::stepSimulation()
{
  auto start_step = std::chrono::high_resolution_clock::now();
  auto do_step = [this]() {
    simStep();
    updateData();
    return controlStep();
  };
  bool done = false;
  if(!config.step_by_step || (config.step_by_step && rem_steps > 0))
  {
    done = do_step();
  }
  if(config.step_by_step && rem_steps == 0 && controller)
  {
    controller->running = false;
    controller->run();
    controller->running = true;
  }
  if(config.sync_real_time)
  {
    std::this_thread::sleep_until(start_step + std::chrono::duration<double, std::micro>(1e6 * model->opt.timestep));
  }
  return done;
}

bool MjSimImpl::render()
{
  if(!config.with_visualization)
  {
    return false;
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
