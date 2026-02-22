# SampleNeckPolicy — NN Policy Controller for JVRC1

A minimal mc_rtc controller that uses a [libtorch](https://pytorch.org/cppdocs/) neural network to drive JVRC1's neck yaw joint. Intended as a starting point for running NN policies inside mc_mujoco.

## What it does

Each control cycle:
1. Advances an internal phase oscillator (configurable `period`)
2. Feeds `[sin(phase), cos(phase)]` into a 2-layer MLP
3. Scales the network output by `amplitude` and sets it as the NECK_Y posture target

The result is the robot smoothly shaking its head.

## File layout

```
sample_controller/
├── CMakeLists.txt              # FetchContent libtorch download, mc_rtc macros
├── etc/
│   └── SampleNeckPolicy.in.yaml  # Controller config (period, amplitude)
└── src/
    ├── CMakeLists.txt           # add_controller(), torch linking, RPATH
    ├── api.h                    # DLL export macros
    ├── controller.h             # SampleNeckPolicy class declaration
    ├── controller.cpp           # Constructor, reset(), run() implementation
    └── lib.cpp                  # CONTROLLER_CONSTRUCTOR registration
```

## Build

The sample controller is built automatically with mc_mujoco (controlled by `BUILD_SAMPLE_CONTROLLER`, default ON):

```bash
cd docker && make run
```

To disable:
```bash
cmake ... -DBUILD_SAMPLE_CONTROLLER=OFF
```

libtorch (CPU, 2.1.0) is auto-downloaded via CMake FetchContent if not already available on the system.

## Configuration

After install, the config lives at `<mc_controller_install_prefix>/etc/SampleNeckPolicy.yaml`:

```yaml
# Oscillation period in seconds
period: 3.0

# Max neck yaw angle in radians
amplitude: 0.5
```

To run it, set your `~/.config/mc_rtc/mc_rtc.yaml`:

```yaml
MainRobot: JVRC1
Timestep: 0.002
Enabled: [SampleNeckPolicy]
```

Then:
```bash
mc_mujoco                          # with MuJoCo visualization
mc_mujoco --without-visualization  # headless
mc_rtc_ticker                      # standalone (no physics)
```

## Threading for larger models

This sample runs torch inference inline in `run()`, which is fine for the tiny dummy MLP. For real policies (e.g. large observation spaces, deep networks, or critic evaluation), the forward pass can take long enough to violate the real-time control deadline. In that case, you should offload inference to a dedicated thread.

The recommended pattern (used by [mc_walker_policy](https://github.com/rohanpsingh/mc_walker_policy)):

1. **Add a worker thread and condition variable** to your controller:
   ```cpp
   // controller.h
   std::thread policy_th_;
   bool policy_th_exit_ = false;
   std::function<void()> * policy_work_ = nullptr;
   std::condition_variable cv_;
   std::mutex mutex_;
   ```

2. **Spawn the thread in the constructor**, waiting for work:
   ```cpp
   // controller.cpp — constructor
   policy_th_ = std::thread([this]()
   {
     while(!policy_th_exit_)
     {
       std::unique_lock<std::mutex> lock(mutex_);
       cv_.wait(lock, [this]{ return policy_work_ != nullptr || policy_th_exit_; });
       if(policy_th_exit_) break;
       (*policy_work_)();
       policy_work_ = nullptr;
       lock.unlock();
       cv_.notify_one();
     }
   });
   ```

3. **Submit work from `run()`** and wait for the result:
   ```cpp
   // controller.cpp — run()
   std::function<void()> work = [&]()
   {
     torch::NoGradGuard no_grad;
     auto out = model_->forward(input);
     output_val = out.item<double>();
   };
   {
     std::unique_lock<std::mutex> lock(mutex_);
     policy_work_ = &work;
   }
   cv_.notify_one();
   {
     std::unique_lock<std::mutex> lock(mutex_);
     cv_.wait(lock, [this]{ return policy_work_ == nullptr; });
   }
   ```

4. **Clean up in the destructor**:
   ```cpp
   // controller.cpp — destructor
   {
     std::lock_guard<std::mutex> lock(mutex_);
     policy_th_exit_ = true;
   }
   cv_.notify_one();
   policy_th_.join();
   ```

This keeps the torch runtime (and any GPU synchronization) off the main control thread. For a full production example with FSM states, safety checks, critic evaluation, and joystick teleoperation, see [mc_walker_policy](https://github.com/rohanpsingh/mc_walker_policy).
