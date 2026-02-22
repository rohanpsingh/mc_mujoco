#pragma once

#include "api.h"

#include <mc_control/mc_controller.h>

#include <torch/torch.h>

struct SampleNeckPolicy_DLLAPI SampleNeckPolicy : public mc_control::MCController
{
  SampleNeckPolicy(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  torch::nn::Sequential model_{nullptr};

  double phase_ = 0.0;
  double period_ = 3.0;
  double amplitude_ = 0.5;
  int jointIndex_ = 0;
};
