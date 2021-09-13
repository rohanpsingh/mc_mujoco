#include "mj_sim.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

//std::mutex mtx;
bool render_state = true;

void simulate(MjSim & mj_sim)
{
  bool done = false;
  while(!done && render_state)
  {
    mj_sim.simStep();
    mj_sim.updateData();
    done = mj_sim.controlStep();
    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int main(int argc, char * argv[])
{
  /* Create a global controller */
  std::string conf_file = "";
  if(argc > 1)
  {
    conf_file = argv[1];
  }

  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_mujoco was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_mujoco",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_control::MCGlobalController controller(conf_file);

  MjConfiguration config;
  auto mj_c = controller.configuration().config("MUJOCO", mc_rtc::Configuration{});
  config.simulationTimestep = controller.timestep();
  mj_c("xmlModelPath", config.xmlPath);
  mj_c("pdGainsPath", config.pdGains);

  MjSim mj_sim(controller, config);
  mj_sim.startSimulation();

  std::thread simThread(simulate, std::ref(mj_sim));

  while (render_state){
    render_state = mj_sim.render();
  }

  simThread.join();
  mj_sim.stopSimulation();
  return 0;
}
