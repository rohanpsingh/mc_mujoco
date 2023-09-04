#pragma once

#include "mj_sim.h"

#include "MujocoClient.h"

#include "mujoco.h"

#ifdef USE_UI_ADAPTER
#  include "platform_ui_adapter.h"
#endif

#include <condition_variable>

namespace mc_mujoco
{

using duration_ms = std::chrono::duration<double, std::milli>;
using duration_us = std::chrono::duration<double, std::micro>;

using clock = std::conditional_t<std::chrono::high_resolution_clock::is_steady,
                                 std::chrono::high_resolution_clock,
                                 std::chrono::steady_clock>;

/** Mujoco free-floating unarticulated objects that do not exist in mc_rtc */
struct MjObject
{
  /** Name in config */
  std::string name;
  /** Initial pose in world frame */
  sva::PTransformd init_pose;
  /** Root body name in MuJoCo */
  std::string root_body;
  /** Root body id */
  int root_body_id = -1;
  /** Free joint in MuJoCo */
  std::string root_joint;
  /** Root joint type */
  mjtJoint root_joint_type = mjJNT_FREE;
  /** Index of robot's root in qpos, -1 if nq == 0 */
  int root_qpos_idx = -1;
  /** Index of robot's root in qvel, -1 if ndof == 0 */
  int root_qvel_idx = -1;
  /** Number of generalized coordinates */
  int nq = -1;
  /** Number of dof */
  int ndof = -1;

  /** Initialize some data after the simulation has started */
  void initialize(mjModel * model);
};

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
  /** Root joint type */
  mjtJoint root_joint_type = mjJNT_FREE;
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

  /** Proportional gains for low-level PD control (read from file) */
  std::vector<double> default_kp = {};
  /** Derivative gains for low-level PD control (read from file) */
  std::vector<double> default_kd = {};
  /** Proportional gains for low-level PD control (used in PD loop) */
  std::vector<double> kp = {};
  /** Derivative gains for low-level PD control (used in PD loop) */
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
  /** Previous torque desired by mc_rtc */
  std::vector<double> mj_prev_ctrl_jointTorque;
  /** Next position desired by mc_rtc */
  std::vector<double> mj_next_ctrl_q;
  /** Next velocity desired by mc_rtc */
  std::vector<double> mj_next_ctrl_alpha;
  /** Next torque desired by mc_rtc */
  std::vector<double> mj_next_ctrl_jointTorque;

  /** Initialize some data after the simulation has started */
  void initialize(mjModel * model, const mc_rbdyn::Robot & robot);

  /** Reset the state based on the mc_rtc robot state */
  void reset(const mc_rbdyn::Robot & robot);

  /** Update sensors based on model and data */
  void updateSensors(mc_control::MCGlobalController * gc, mjModel * model, mjData * data);

  /** Update the control */
  void updateControl(const mc_rbdyn::Robot & robot);

  /** Send control to MuJoCo */
  void sendControl(const mjModel & model, mjData & data, size_t interp_idx, size_t frameskip_, bool torque_control);

  /** Run PD control for a given joint */
  double PD(size_t jnt_id, double q_ref, double q, double qdot_ref, double qdot);

  /** Load PD gains from a file */
  bool loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints);

  /** From a name returns the prefixed name in MuJoCo */
  inline std::string prefixed(const std::string & name) const noexcept
  {
    if(!prefix.empty())
    {
      return fmt::format("{}_{}", prefix, name);
    }
    return name;
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

#ifndef USE_UI_ADAPTER
  /** GLFW window, might be null if the visualization is disabled */
  GLFWwindow * window = nullptr;

  /** GPU context */
  mjrContext context;

  /** Keyboard and mouse states */
  mjuiState uistate;
#else
  /** Platform UI adapter */
  std::unique_ptr<mujoco::PlatformUIAdapter> platform_ui_adapter;
#endif

  /** Camera */
  mjvCamera camera;

  /** Visualization options */
  mjvOption options;

  /** Visualization scene */
  mjvScene scene;

  /** Mouse perturbations */
  mjvPerturb pert;

  /** Start of the previous iteration */
  clock::time_point mj_sim_start_t;
  /** Accumulated delay to catch up to real-time performace */
  duration_us mj_sync_delay = duration_us(0);
  /** Time taken for the last 1024 iterations */
  std::array<double, 1024> mj_sim_dt;
  /** Average of the last 1024 iterations */
  double mj_sim_dt_average;

  /** Number of steps left to play in step by step mode */
  size_t rem_steps = 0;

  /** Robots in simulation and mc_rtc */
  std::vector<MjRobot> robots;

  /** Objects in simulation */
  std::vector<MjObject> objects;

  /*! Simulation wall clock time (seconds) */
  double wallclock;

private:
  /** Number of MuJoCo iteration since the start */
  size_t iterCount_ = 0;
  /** How often we run mc_rtc relative to MuJoCo physics */
  size_t frameskip_ = 1;

  /** True if the simulation should be reset on the next step */
  bool reset_simulation_ = false;

  /** Mutex used in rendering */
  std::mutex rendering_mutex_;

  template<typename T>
  void setPosW(const T & robot, const sva::PTransformd & pos);

public:
  MjSimImpl(const MjConfiguration & config);

  void cleanup();

  void makeDatastoreCalls();

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

  // Set the position of an object
  // No-op if the object is not in the simulation
  void setObjectPosW(const std::string & object, const sva::PTransformd & pt);

  // Set the position of a robot
  // No-op if the robot is not in the controller
  void setRobotPosW(const std::string & robot, const sva::PTransformd & pt);

  void setSimulationInitialState();

  void saveGUISettings();

  void loadPlugins(const mc_rtc::Configuration & mc_mujoco_cfg) const;

  inline mc_control::MCGlobalController * get_controller() noexcept
  {
    return controller.get();
  }

  std::map<std::string, std::vector<double>> init_qs_;
  std::map<std::string, sva::PTransformd> init_pos_;
};

} // namespace mc_mujoco
