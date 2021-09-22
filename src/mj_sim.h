#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include "mj_configuration.h"

namespace mc_mujoco
{

struct MjSimImpl;

struct MjSim
{
public:
  /*! \brief Constructor
   *
   * Prepare to start a simulation.
   *
   * \param config Configuration for mc_mujoco
   *
   * \param mc_config Configuration file used by mc_rtc
   *
   */
  MjSim(const MjConfiguration & config);

  /*! \brief Destructor */
  ~MjSim();

  /** Plays one step of physics simulation, should be called as often as possible
   *
   * \returns True if the controller fails and the simulation should stop
   */
  bool stepSimulation();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Update the GUI, no-op if visualization is disabled
   *
   * \returns True if the window was closed by the user
   */
  bool render();

private:
  std::unique_ptr<MjSimImpl> impl;
};

} // namespace mc_mujoco
