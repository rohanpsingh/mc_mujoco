# Sample controllers

This directory holds example mc_rtc controllers that ship with mc_mujoco. Two
flavours coexist:

- **In-tree** (e.g. `neck_policy/`) — small, dependency-light samples that live
  directly in the mc_mujoco source tree.
- **Submodule** (e.g. `grasp-fsm/`) — larger samples maintained in their own
  repositories and pulled in via `git submodule`.

Adding a new sample of either kind only requires placing it here:
`examples/CMakeLists.txt` auto-discovers any subdirectory containing a
`CMakeLists.txt` and adds it to the build (gated by `-DBUILD_EXAMPLES=ON`,
which is the default). Empty (un-init'd) submodule directories are silently
skipped, so a fresh clone without `git submodule update --init` still
configures cleanly.

## Running a sample

`make run` builds every available sample alongside mc_mujoco core. Each sample
ships its own `etc/mc_rtc.in.yaml` (and optionally `etc/mc_mujoco.in.yaml` for
scene objects). To activate one inside the container:

```sh
cp /workspace/mc_mujoco/examples/<name>/etc/mc_rtc.in.yaml \
   ~/.config/mc_rtc/mc_rtc.yaml
# if the sample needs scene objects:
mkdir -p ~/.config/mc_rtc/mc_mujoco
cp /workspace/mc_mujoco/examples/<name>/etc/mc_mujoco.in.yaml \
   ~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml
mc_mujoco
```

The default `mc_rtc.yaml` written by `entrypoint.sh` (only created if absent)
runs `Posture, EndEffector, CoM` against `JVRC1` — the user's choice survives
subsequent `make run` invocations.

## Dependency pinning

Sample controllers may depend on mc_rtc-ecosystem libraries (e.g.
`baseline_walking_controller`) that mc_mujoco vendors into the Docker image.
Those library pins live in `examples/MC_RTC_DEPS_VERSION` and the
`docker/Dockerfile` layers it gates. Bump the sentinel to invalidate the
layer cache after editing the Dockerfile pins.
