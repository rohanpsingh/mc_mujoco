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

  /** Reset the simulation and controller to the initial state */
  void resetSimulation();

  /** Plays one step of physics simulation, should be called as often as possible
   *
   * \returns True if the controller fails and the simulation should stop
   */
  bool stepSimulation();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Prepare to render */
  void updateScene();

  /*! Update the GUI, no-op if visualization is disabled
   *
   * \returns False if the application should quit
   */
  bool render();

  /** The underlying global controller instance in the simulation
   *
   * nullptr if with_controller was false in MjConfiguration
   */
  mc_control::MCGlobalController * controller() noexcept;

private:
  std::unique_ptr<MjSimImpl> impl;
};

} // namespace mc_mujoco
