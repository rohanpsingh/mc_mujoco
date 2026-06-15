#pragma once

namespace mc_mujoco
{

/** Default key path */
constexpr auto MUJOCO_KEY_PATH = "@MUJOCO_ROOT_DIR@/bin/mjkey.txt";

/** System folder searched for Mujoco models */
constexpr auto SHARE_FOLDER = "@MC_MUJOCO_SHARE_DESTINATION@";

/** User folder searched for Mujoco models */
constexpr auto USER_FOLDER = "@MC_MUJOCO_USER_DESTINATION@";

} // namespace mc_mujoco
