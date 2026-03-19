#pragma once

#include <type_traits>

namespace mc_mujoco
{

enum class ControlAxis
{
  NONE = 0,
  TX = (1u << 0),
  TY = (1u << 1),
  TZ = (1u << 2),
  RX = (1u << 3),
  RY = (1u << 4),
  RZ = (1u << 5),
  TRANSLATION = TX | TY | TZ,
  ROTATION = RX | RY | RZ,
  XYTHETA = TX | TY | RZ,
  XYZTHETA = TX | TY | TZ | RZ,
  ALL = TRANSLATION | ROTATION
};

inline ControlAxis operator|(ControlAxis lhs, ControlAxis rhs)
{
  using enum_t = std::underlying_type_t<ControlAxis>;
  return static_cast<ControlAxis>(static_cast<enum_t>(lhs) | static_cast<enum_t>(rhs));
}

inline ControlAxis operator&(ControlAxis lhs, ControlAxis rhs)
{
  using enum_t = std::underlying_type_t<ControlAxis>;
  return static_cast<ControlAxis>(static_cast<enum_t>(lhs) & static_cast<enum_t>(rhs));
}

} // namespace mc_mujoco
