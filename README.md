# Mujoco interface for mc-rtc

### Usage

Assuming that you have mujoco installed under `${HOME}/.mujoco/mujoco212`,

```sh
$ git clone --recursive git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=$HOME/.mujoco/mujoco212
$ make
$ make install
```
Add the following line to your `~/.bashrc`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${HOME}/.mujoco/mujoco212/lib:${HOME}/.mujoco/mujoco212/bin
```
Then, to run the interface:
```sh
$ cd mc_mujoco/build/
$ ./src/mc_mujoco
```
---

 
- (For old versions of mujoco) By default, we assume your mujoco key is at `${MUJOCO_ROOT_DIR}/bin/mjkey.txt` if that is not the case you can set the environment variabe `MUJOCO_KEY_PATH` to the path where your `mjkey.txt` is.
- If Mujoco simulation becomes unstable `WARNING: Nan, Inf or huge value ... The simulation is unstable`. Try setting the mc-rtc control timestep lower (`0.001` or `0.002`).

### GUI

**Apply external force**

An object is selected by left-double-click. The user can then apply forces and torques on the selected object by holding `Ctrl` key and dragging the mouse.

### Credits

This package includes code from:
- [imgui v1.84.2](https://github.com/ocornut/imgui/)
- [implot v0.11](https://github.com/epezent/implot)
- [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo)
- [pugixml v1.11.4](https://github.com/zeux/pugixml)
- [Roboto font](https://github.com/googlefonts/roboto)
