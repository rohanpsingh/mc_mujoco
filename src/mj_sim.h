#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

namespace mc_mujoco
{

struct MjSimImpl;

/** Configuration for the connection to MuJoCo and the simulation */
struct MjConfiguration
{
};

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
  MjSim(const MjConfiguration & config, const std::string & mc_config = "");

  /*! \brief Destructor */
  ~MjSim();

  /*! \brief Start the simulation. This should be called
   * only once
   */
  void startSimulation();

  /*! Trigger the next simulation step. This should be
   * called as long as the simulation is running.
   */
  bool controlStep();

  /*! Trigger the next simulation step. This should be
   * called as long as the simulation is running.
   */
  void simStep();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Read sim state and set in controller */
  void updateData();

  /*! Update the GUI */
  bool render();

private:
  std::unique_ptr<MjSimImpl> impl;
};

} // namespace mc_mujoco
