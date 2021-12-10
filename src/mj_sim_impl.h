#pragma once

#include "mj_sim.h"

#include "MujocoClient.h"

#include "mujoco.h"

#include <condition_variable>

namespace mc_mujoco
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                                 std::chrono::high_resolution_clock,
                                 std::chrono::steady_clock>;

/** Interface between a Mujoco robot and an mc_rtc robot */
struct MjRobot
{
  /** Name in mc_rtc */
  std::string name;
  /** Prefix in MuJoCo */
  std::string prefix;
  /** Root body name in MuJoCo */
  std::string root_body;
  /** Root body id */
  int root_body_id = -1;
  /** Free joint in MuJoCo */
  std::string root_joint;
  /** Index of robot's root in qpos, -1 if fixed base */
  int root_qpos_idx = -1;
  /** Index of robot's root in qvel, -1 if fixed base */
  int root_qvel_idx = -1;
  /** Position of FloatingBase sensor */
  Eigen::Vector3d root_pos;
  /** Orientation of FloatingBase sensor */
  Eigen::Quaterniond root_ori;
  /** Linear velocity of FloatingBase sensor */
  Eigen::Vector3d root_linvel;
  /** Angular velocity of FloatingBase sensor */
  Eigen::Vector3d root_angvel;
  /** Linear acceleration of FloatingBase sensor */
  Eigen::Vector3d root_linacc;
  /** Angular acceleration of FloatingBase sensor */
  Eigen::Vector3d root_angacc;
  /** Encoders in robot.ref_joint_order */
  std::vector<double> encoders;
  /** Joints' velocity in robot.ref_joint_order */
  std::vector<double> alphas;
  /** Joints' torque in robot.ref_joint_order */
  std::vector<double> torques;
  /** Force sensors reading */
  std::map<std::string, sva::ForceVecd> wrenches;
  /** Gyro readings */
  std::map<std::string, Eigen::Vector3d> gyros;
  /** Accelerometer readings */
  std::map<std::string, Eigen::Vector3d> accelerometers;

  /** Proportional gains for low-level PD control */
  std::vector<double> kp = {};
  /** Derivative gains for low-level PD control */
  std::vector<double> kd = {};

  /** Names of the motors inside MuJoCo corresponding to the joints in \ref mj_jnt_names the name is empty if the joint
   * has no motor actuation in MuJoCo */
  std::vector<std::string> mj_mot_names;
  /** Correspondance from motor name to id inside MuJoCo */
  std::vector<int> mj_mot_ids;
  /** Names of the position actuators inside MuJoCo */
  std::vector<std::string> mj_pos_act_names;
  /** Corresppondance from position actuator to id inside MuJoCo */
  std::vector<int> mj_pos_act_ids;
  /** Names of the velocity actuators inside MuJoCo */
  std::vector<std::string> mj_vel_act_names;
  /** Corresppondance from velocity actuator to id inside MuJoCo */
  std::vector<int> mj_vel_act_ids;
  /** Names of the joints inside MuJoCo */
  std::vector<std::string> mj_jnt_names;
  /** Correspondance from joint name to id inside MuJoCo */
  std::vector<int> mj_jnt_ids;
  /** MuJoCo joint to rjo index */
  std::vector<int> mj_jnt_to_rjo;
  /** Correspondance from mc_rtc force sensor's name to MuJoCo force sensor id, -1 if absent */
  std::unordered_map<std::string, int> mc_fs_to_mj_fsensor_id;
  /** Correspondance from mc_rtc force sensor's name to MuJoCo torque sensor id, -1 if absent */
  std::unordered_map<std::string, int> mc_fs_to_mj_tsensor_id;
  /** Correspondance from mc-rtc body sensor's name to MuJoCo gyro sensor id, -1 if absent */
  std::unordered_map<std::string, int> mc_bs_to_mj_gyro_id;
  /** Correspondance from mc-rtc body sensor's name to MuJoCo accelerometer sensor id, -1 if absent */
  std::unordered_map<std::string, int> mc_bs_to_mj_accelerometer_id;

  /** Transform from index in mj_mot_names to index in mbc, -1 if not in mbc */
  std::vector<int> mj_to_mbc;
  /** Command send to mujoco */
  std::vector<double> mj_ctrl;
  /** Previous position desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_q;
  /** Previous velocity desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_alpha;
  /** Next position desired by mc_rtc */
  std::vector<double> mj_next_ctrl_q;
  /** Next velocity desired by mc_rtc */
  std::vector<double> mj_next_ctrl_alpha;

  /** Initialize some data after the simulation has started */
  void initialize(mjModel * model, const mc_rbdyn::Robot & robot);

  /** Reset the state based on the mc_rtc robot state */
  void reset(const mc_rbdyn::Robot & robot);

  /** Update sensors based on model and data */
  void updateSensors(mc_control::MCGlobalController * gc, mjModel * model, mjData * data);

  /** Update the control */
  void updateControl(const mc_rbdyn::Robot & robot);

  /** Send control to MuJoCo */
  void sendControl(const mjModel & model, mjData & data, size_t interp_idx, size_t frameskip_);

  /** Run PD control for a given joint */
  double PD(double jnt_id, double q_ref, double q, double qdot_ref, double qdot);

  /** Load PD gains from a file */
  bool loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints);

  /** From a name returns the prefixed name in MuJoCo */
  inline std::string prefixed(const std::string & name) const noexcept
  {
    if(prefix.size())
    {
      return fmt::format("{}_{}", prefix, name);
    }
    else
    {
      return name;
    }
  }
};

struct MjSimImpl
{
private:
  /** Controller instance in this simulation, might be null if the controller is disabled */
  std::unique_ptr<mc_control::MCGlobalController> controller;

public:
  /** Configuration and data for the step-by-step mode */
  MjConfiguration config;

  /** Client instance in this simulation, might be null if the visualization is disabled */
  std::unique_ptr<MujocoClient> client;

  /** MuJoCo model */
  mjModel * model = nullptr;

  /** MuJoCo data */
  mjData * data = nullptr;

  /** Initial state */
  std::vector<double> qInit;

  /** Initial velocity */
  std::vector<double> alphaInit;

  /** GLFW window, might be null if the visualization is disabled */
  GLFWwindow * window = nullptr;

  /** Camera */
  mjvCamera camera;

  /** Visualization options */
  mjvOption options;

  /** Visualization scene */
  mjvScene scene;

  /** Mouse perturbations */
  mjvPerturb pert;

  /** GPU context */
  mjrContext context;

  /** Keyboard and mouse states */
  mjuiState uistate;

  /** Start of the previous iteration */
  clock::time_point mj_sim_start_t;
  /** Accumulated delay to catch up to real-time performace */
  duration_us mj_sync_delay = duration_us(0);
  /** Time taken for the last 1024 iterations */
  std::array<double, 1024> mj_sim_dt;
  /** Average of the last 1024 iterations */
  double mj_sim_dt_average;

  /** Robots in simulation and mc_rtc */
  std::vector<MjRobot> robots;

private:
  /** Number of MuJoCo iteration since the start */
  size_t iterCount_ = 0;
  /** How often we run mc_rtc relative to MuJoCo physics */
  size_t frameskip_ = 1;

  /** Number of steps left to play in step by step mode */
  size_t rem_steps = 0;

  /** True if the simulation should be reset on the next step */
  bool reset_simulation_ = false;

  /** Mutex used in rendering */
  std::mutex rendering_mutex_;

  /** Condition variable used to communicate between the physics and the rendered */
  std::condition_variable rendering_cv_;

public:
  MjSimImpl(const MjConfiguration & config);

  void cleanup();

  void startSimulation();

  void updateData();

  bool controlStep();

  void simStep();

  bool stepSimulation();

  void updateScene();

  bool render();

  void stopSimulation();

  void resetSimulation(const std::map<std::string, std::vector<double>> & reset_qs,
                       const std::map<std::string, sva::PTransformd> & reset_pos);

  void setSimulationInitialState();

  void saveGUISettings();

  inline mc_control::MCGlobalController * get_controller() noexcept
  {
    return controller.get();
  }
};

} // namespace mc_mujoco
