#include "controller.h"

SampleNeckPolicy::SampleNeckPolicy(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config)
{
  // Read config
  if(config.has("period"))
  {
    period_ = config("period");
  }
  if(config.has("amplitude"))
  {
    amplitude_ = config("amplitude");
  }

  // Setup constraints and posture task (standard mc_rtc pattern)
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);

  // Build a small dummy MLP: 2 inputs (sin, cos) -> 64 -> 64 -> 1 output
  model_ = torch::nn::Sequential(
      torch::nn::Linear(2, 64),
      torch::nn::Tanh(),
      torch::nn::Linear(64, 64),
      torch::nn::Tanh(),
      torch::nn::Linear(64, 1),
      torch::nn::Tanh());
  model_->eval();

  // Find NECK_Y joint index in posture task ordering
  const auto & robot = this->robot();
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    if(rjo[i] == "NECK_Y")
    {
      jointIndex_ = static_cast<int>(i);
      break;
    }
  }

  mc_rtc::log::info("[SampleNeckPolicy] init done — period={}, amplitude={}, NECK_Y index={}", period_, amplitude_,
                     jointIndex_);
}

void SampleNeckPolicy::reset(const mc_control::ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  phase_ = 0.0;
}

bool SampleNeckPolicy::run()
{
  // Advance phase
  phase_ += timeStep / period_ * 2.0 * M_PI;
  if(phase_ > 2.0 * M_PI)
  {
    phase_ -= 2.0 * M_PI;
  }

  // Prepare NN input: [sin(phase), cos(phase)]
  torch::NoGradGuard no_grad;
  auto input = torch::tensor({std::sin(phase_), std::cos(phase_)}).unsqueeze(0);

  // Forward pass
  auto out = model_->forward(input);
  double output_val = out.item<double>();

  // Scale by amplitude and set posture target for NECK_Y
  double target = output_val * amplitude_;
  auto posture = postureTask->posture();
  posture[robot().jointIndexByName("NECK_Y")] = {target};
  postureTask->posture(posture);

  return MCController::run();
}
