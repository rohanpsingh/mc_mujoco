#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

// Replaced boost/program_options.hpp with CLI11
#include <CLI/CLI.hpp>

bool render_state = true;

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

void scanPluginLibraries()
{
// define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  const std::string plugin_dir = MUJOCO_BIN_DIR + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(),
      +[](const char * filename, int first, int count)
      {
        std::printf("Plugins registered by library '%s':\n", filename);
        for(int i = first; i < first + count; ++i)
        {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

void simulate(mc_mujoco::MjSim & mj_sim)
{
  bool done = false;
  while(!done && render_state)
  {
    mj_sim.stepSimulation();
  }
}

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_mujoco was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_mujoco",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_mujoco::MjConfiguration config;

  // Initialize CLI11 App
  CLI::App app{"mc_mujoco options"};

  // Positional + Option flag for configuration
  app.add_option("mc-config,-f,--mc-config", config.mc_config, "Configuration given to mc_rtc");

  // Standard boolean switches
  app.add_flag("--step-by-step", config.step_by_step, "Start the simulation in step-by-step mode");
  app.add_flag("--torque-control", config.torque_control, "Enable torque control");
  app.add_flag("-s,--sync", config.sync_real_time, "Synchronize mc_mujoco simulation time with real time");

  // Inverted flags logic (CLI11 handles parsing directly into existing config variables)
  app.add_flag("--without-controller{false}", config.with_controller, "Disable mc_rtc controller inside mc_mujoco");
  app.add_flag("--without-visualization{false}", config.with_visualization, "Disable mc_mujoco GUI");
  app.add_flag("--without-mc-rtc-gui{false}", config.with_mc_rtc_gui, "Disable mc_rtc GUI");
  app.add_flag("--without-visuals{false}", config.visualize_visual, "Disable visuals display");

  // Standard flag for turning a feature ON
  app.add_flag("--with-collisions", config.visualize_collisions, "Visualize collisions model");

  // Parse arguments. CLI11 automatically catches parsing exceptions and `--help` / `-h`.
  CLI11_PARSE(app, argc, argv);

  scanPluginLibraries();

  mc_mujoco::MjSim mj_sim(config);

  std::thread simThread(simulate, std::ref(mj_sim));

  while(render_state)
  {
    mj_sim.updateScene();
    render_state = mj_sim.render();
  }

  simThread.join();
  mj_sim.stopSimulation();
  return 0;
}
