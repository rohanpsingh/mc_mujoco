#pragma once

#include "mj_sim.h"

#include "MujocoClient.h"

#include "mujoco.h"

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
  /** Index of robot's root in qpos */
  size_t root_qpos_idx = 0;
  /** Index of robot's root in qvel */
  size_t root_qvel_idx = 0;
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

  /** Proportional gains for low-level PD control */
  std::vector<double> kp = {};
  /** Derivative gains for low-level PD control */
  std::vector<double> kd = {};

  /** Names of the motors inside MuJoCo */
  std::vector<std::string> mj_mot_names;
  /** Correspondance from motor name to id inside MuJoCo */
  std::vector<int> mj_mot_ids;
  /** Names of the joints inside MuJoCo */
  std::vector<std::string> mj_jnt_names;
  /** Correspondance from joint name to id inside MuJoCo */
  std::vector<int> mj_jnt_ids;

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

  /** GLFW window, might be null if the visualization is disabled */
  GLFWwindow * window = nullptr;

  /** Camera */
  mjvCamera camera;

  /** Visualization options */
  mjvOption options;

  /** Visualization scene */
  mjvScene scene;

  /** GPU context */
  mjrContext context;

  /* Mouse interaction */
  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;

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

public:
  MjSimImpl(const MjConfiguration & config);

  void cleanup();

  void startSimulation();

  void updateData();

  bool controlStep();

  void simStep();

  bool stepSimulation();

  bool render();

  void stopSimulation();
};

} // namespace mc_mujoco
