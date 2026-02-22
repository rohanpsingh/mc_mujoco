# Docker environment

The Docker environment provides a pre-built base with **ROS 2 Humble**, **mc_rtc** (built from source), and **MuJoCo**. It's useful for running `mc_mujoco` without a host install, developing `mc_mujoco` itself, or using it as a base for your own `mc_rtc` controllers.

**Prerequisites:** Docker with Compose v2 (Docker ≥ 20.10), and an X11 display for the GUI.

---

## Running mc_mujoco

```sh
cd docker
make run
```

This will:
1. Build the environment image on first run (slow — downloads and compiles `mc_rtc`)
2. Mount the local source into the container
3. Configure and build `mc_mujoco` with Ninja
4. Drop you into a bash shell inside the container

Then run the simulator:

```sh
mc_mujoco
```

The image is cached after the first build. Subsequent `make run` calls are fast unless `MC_RTC_VERSION`, `MUJOCO_VERSION`, or `docker/Dockerfile` change.

---

## Developing mc_mujoco

The repo is live-mounted at `/workspace/mc_mujoco` inside the container. Edit files on your host normally. To rebuild, first enter the container via `make run`, then from the shell it drops you into:

```sh
cmake --build /workspace/mc_mujoco/build
```

Build artifacts are stored in a Docker-managed volume, so incremental builds persist across container restarts without cluttering your working tree.

To reconfigure (e.g. after adding a CMake option), just re-run `make run`. It reruns cmake configure before dropping you into the shell.

---

## Using the image for your own mc_rtc controller

After `make run` has been run once, the image `mc-rtc-mujoco:latest` contains a full `mc_rtc` + MuJoCo environment. You can use it as a base for your own controller:

```dockerfile
FROM mc-rtc-mujoco:latest

COPY . /workspace/my_controller
RUN cmake -S /workspace/my_controller -B /workspace/my_controller/build -G Ninja \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    cmake --build /workspace/my_controller/build && \
    cmake --install /workspace/my_controller/build
```

Or mount your controller source the same way `mc_mujoco` does: add a bind mount and build volume to a new `docker-compose.yml` that extends `docker-compose.yml`.

---

## Running CI locally

```sh
cd docker
make ci-test
```

Builds `mc_mujoco` from scratch inside the CI container and runs the standalone CMake test suite — the same steps as the GitHub Actions workflow. Build artifacts go into `/ci-build` inside the container and are discarded when it exits.

---

## Version management

| File | Controls |
|------|----------|
| `MUJOCO_VERSION` | MuJoCo version downloaded and installed |
| `MC_RTC_VERSION` | Branch, tag, or commit SHA of mc_rtc built from source |

Changing either file invalidates the corresponding Docker layer cache and triggers a rebuild of that layer (and everything after it) on the next `make run` or `make ci-test`.

To pin `mc_rtc` to a specific commit for reproducibility:

```sh
echo "abc1234" > MC_RTC_VERSION
```
