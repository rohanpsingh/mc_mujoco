#pragma once

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/config.h>

struct MjSimImpl;

/** Configuration for the connection to MuJoCo and the simulation */
struct MjConfiguration
{
  double simulationTimestep = -1;
  std::string xmlPath = "";
  std::string pdGains = "";
};

struct MjSim
{
public:
  /*! \brief Constructor
   *
   * Prepare to start a simulation.
   *
   * \param controller The mc_rtc controller instance used in the simulation.
   * \param config loaded user configuration.
   *
   */
  MjSim(mc_control::MCGlobalController & controller, const MjConfiguration & config);

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
