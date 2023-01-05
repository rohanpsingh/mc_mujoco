#include <mc_mujoco/mj_sim.h>

int main()
{
  mc_mujoco::MjConfiguration config;
  config.with_visualization = false;
  mc_mujoco::MjSim simulation(config);
  auto & ctl = *simulation.controller();
  mc_rtc::log::info("Start pos: {}", ctl.controller().robot().posW().translation().transpose());
  for(size_t i = 0; i < 1000; ++i)
  {
    simulation.stepSimulation();
  }
  mc_rtc::log::info("Final pos: {}", ctl.controller().robot().posW().translation().transpose());
  return 0;
}
