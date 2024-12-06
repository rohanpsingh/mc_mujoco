#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

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
  {
    po::options_description desc("mc_mujoco options");
    po::positional_options_description p;
    p.add("mc-config", 1);
    // clang-format off
    desc.add_options()
      ("help", "Show this help message")
      ("mc-config", po::value<std::string>(&config.mc_config), "Configuration given to mc_rtc")
      ("step-by-step", po::bool_switch(&config.step_by_step), "Start the simulation in step-by-step mode")
      ("torque-control", po::bool_switch(&config.torque_control), "Enable torque control")
      ("without-controller", po::bool_switch(), "Disable mc_rtc controller inside mc_mujoco")
      ("without-visualization", po::bool_switch(), "Disable mc_mujoco GUI")
      ("without-mc-rtc-gui", po::bool_switch(), "Disable mc_rtc GUI")
      ("with-collisions", po::bool_switch(), "Visualize collisions model")
      ("without-visuals", po::bool_switch(), "Disable visuals display")
      ("sync", po::bool_switch(&config.sync_real_time), "Synchronize mc_mujoco simulation time with real time");
    // clang-format on
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
      std::cout << desc << "\n";
      return 0;
    }
    config.with_controller = !vm["without-controller"].as<bool>();
    config.with_visualization = !vm["without-visualization"].as<bool>();
    config.with_mc_rtc_gui = !vm["without-mc-rtc-gui"].as<bool>();
    if(!vm["without-visuals"].defaulted())
    {
      config.visualize_visual = !vm["without-visuals"].as<bool>();
    }
    if(!vm["with-collisions"].defaulted())
    {
      config.visualize_collisions = vm["with-collisions"].as<bool>();
    }
  }

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
