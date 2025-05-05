#pragma once

#include <optional>
#include <string>

namespace mc_mujoco
{

/** Configuration for the connection to MuJoCo and the simulation */
struct MjConfiguration
{
  /** If true, enable visualization */
  bool with_visualization = true;
  /** If true, display the collision model by default */
  std::optional<bool> visualize_collisions;
  /** If true, display the visual model by default */
  std::optional<bool> visualize_visual;
  /** If true, enable mc_rtc GUI inside MuJoCo simulation */
  bool with_mc_rtc_gui = true;
  /** If true, enable mc_rtc controller inside MuJoCo simulation */
  bool with_controller = true;
  /** If true, sync simulation time and real time */
  bool sync_real_time = false;
  /** If true, start in step-by-step mode */
  bool step_by_step = false;
  /** mc_rtc configuration file */
  std::string mc_config = "";
  /** Use torque-control rather than position control */
  bool torque_control = false;
  /** Freeze root joints of all mc_rtc robots by increasing damping and removing ground */
  bool fix_base_link = false;
};

} // namespace mc_mujoco
