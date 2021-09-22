#pragma once

#include "mj_sim.h"

#include "MujocoClient.h"

#include "mujoco.h"

namespace mc_mujoco
{

struct MjSimImpl
{
private:
  /** Controller instance in this simulation, might be null if the controller is disabled */
  std::unique_ptr<mc_control::MCGlobalController> controller;

public:
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

private:
  /** Number of MuJoCo iteration since the start */
  size_t iterCount_ = 0;
  /** How often we run mc_rtc relative to MuJoCo physics */
  size_t frameskip_ = 1;

  /** Position of FloatingBase sensor */
  Eigen::Vector3d root_pos;
  /** Orientation of FloatingBase sensor */
  Eigen::Quaterniond root_orient;
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

  /** PD Control */
  double PD(double jnt_id, double q_ref, double q, double qdot_ref, double qdot);

  /** Load PD gains from a file */
  bool loadGain(const std::string & path_to_pd, const std::vector<std::string> & joints);

public:
  MjSimImpl(const MjConfiguration & config, const std::string & mc_config);

  void cleanup();

  void startSimulation();

  void updateData();

  bool controlStep();

  void simStep();

  bool render();

  void stopSimulation();
};

} // namespace mc_mujoco
