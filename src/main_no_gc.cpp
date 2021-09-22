#include "mj_utils.h"

#include <mc_rtc/logging.h>

#include <thread>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    mc_rtc::log::critical("[usage] {} [model.xml]", argv[0]);
    return 1;
  }
  bool initialized = mc_mujoco::mujoco_init(argv[1]);
  if(!initialized)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_mujoco] Initialized failed.");
  }
  mc_mujoco::mujoco_create_window();
  mc_mujoco::mujoco_step();
  while(mc_mujoco::mujoco_render())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  mc_mujoco::mujoco_cleanup();
  return 0;
}
