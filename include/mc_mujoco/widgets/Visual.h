#pragma once

#include "MujocoWidget.h"

namespace mc_mujoco
{

struct Visual : public MujocoWidget
{
  Visual(Client & client, const ElementId & id);

  void data(const rbd::parsers::Visual & visual, const sva::PTransformd & pos);

  void draw2D() override;

  void draw3D() override;

private:
  rbd::parsers::Visual visual_;
  sva::PTransformd pos_;
  bool show_ = true;
};

} // namespace mc_mujoco
