# [WIP] Mujoco interface for mc-rtc

Add this to your mc-rtc configuration yaml:

```yaml
MUJOCO:
  xmlModelPath: "<path_to_your_xml_model>"
  pdGainsPath: "<path_to_your_pdgains>"
```

### Usage

Assuming that you have mujoco installed under `${HOME}/.mujoco/mujoco200`,

```sh
$ git clone git@github.com:rohanpsingh/mc_mujoco.git
$ cd mc_mujoco
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMUJOCO_ROOT_DIR=$HOME/.mujoco/mujoco200
$ make
```

Then, to run the interface:
```sh
$ cd mc_mujoco/build/
$ ./src/mc_mujoco
```
---

- By default, we assume your mujoco key is at `${MUJOCO_ROOT_DIR}/bin/mjkey.txt` if that is not the case you can set the environment variabe `MUJOCO_KEY_PATH` to the path where your `mjkey.txt` is.
- If Mujoco simulation becomes unstable `WARNING: Nan, Inf or huge value ... The simulation is unstable`. Try setting the mc-rtc control timestep lower (`0.001` or `0.002`).


### Credits

This package includes code from:
- [imgui v1.84.2](https://github.com/ocornut/imgui/)
- [implot v0.11](https://github.com/epezent/implot)
- [Roboto font](https://github.com/googlefonts/roboto)
